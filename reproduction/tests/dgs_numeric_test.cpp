#include "optimizationRound.h"
#include <distributed_mapper/distributed_mapper.h>
#include <distributed_mapper/distributed_mapper_utils.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <boost/make_shared.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <vector>

using namespace gtsam;
using dcl_slam::OptimizationRound;
using dcl_slam::optimization_state;
using dcl_slam::neighbor_estimate;
using dcl_slam::neighbor_estimateConstPtr;

struct GraphCase {
    std::string name;
    int num_robots;
    std::vector<NonlinearFactorGraph::shared_ptr> subgraphs;
    std::vector<Values::shared_ptr> initials;
    NonlinearFactorGraph full_graph;
    Key anchor_key;
    Pose3 anchor_pose;
    SharedNoiseModel prior_noise;
};

static GraphCase makeFreezeCounterexample() {
    GraphCase c;
    c.name = "3-robot sweep-1 freeze counterexample";
    c.num_robots = 3;
    c.prior_noise = noiseModel::Isotropic::Variance(6, 1e-12);
    auto noise = noiseModel::Isotropic::Sigma(6, 0.05);

    std::vector<Pose3> p_init = {
        Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0)),
        Pose3(Rot3::RzRyRx(0.0, 0.0, 0.6), Point3(3.0, 0.0, 0.0)),
        Pose3(Rot3::RzRyRx(0.0, 0.0, 1.2), Point3(1.5, 2.5, 0.0)),
    };
    c.anchor_key = Symbol('a', 0);
    c.anchor_pose = p_init[0];
    c.full_graph.add(PriorFactor<Pose3>(c.anchor_key, c.anchor_pose, c.prior_noise));
    for (int r = 0; r < 3; ++r) {
        c.subgraphs.push_back(boost::make_shared<NonlinearFactorGraph>());
        c.initials.push_back(boost::make_shared<Values>());
        Key k0 = Symbol('a' + r, 0), k1 = Symbol('a' + r, 1);
        Pose3 delta(Rot3::RzRyRx(0.0, 0.0, 0.1), Point3(1.0, 0.0, 0.0));
        c.initials[r]->insert(k0, p_init[r]);
        c.initials[r]->insert(k1, p_init[r] * delta);
        auto f = boost::make_shared<BetweenFactor<Pose3>>(k0, k1, delta, noise);
        c.subgraphs[r]->push_back(f);
        c.full_graph.push_back(f);
    }
    std::vector<std::tuple<Key, Key, Pose3>> inter = {
        {Symbol('a', 1), Symbol('b', 0), Pose3(Rot3::RzRyRx(0.0, 0.0, 0.35), Point3(1.8, 0.2, 0.0))},
        {Symbol('b', 1), Symbol('c', 0), Pose3(Rot3::RzRyRx(0.0, 0.0, 0.45), Point3(0.5, 1.6, 0.0))},
        {Symbol('c', 1), Symbol('a', 0), Pose3(Rot3::RzRyRx(0.0, 0.0, -0.9), Point3(-1.2, -2.0, 0.0))},
    };
    for (const auto& t : inter) {
        Key k0 = std::get<0>(t), k1 = std::get<1>(t);
        auto f = boost::make_shared<BetweenFactor<Pose3>>(k0, k1, std::get<2>(t), noise);
        c.subgraphs[OptimizationRound::robot(k0)]->push_back(f);
        c.subgraphs[OptimizationRound::robot(k1)]->push_back(f);
        c.full_graph.push_back(f);
    }
    return c;
}

static GraphCase makeNoisyCycle3(unsigned seed) {
    GraphCase c;
    c.name = "3-robot noisy cycle (seed " + std::to_string(seed) + ")";
    c.num_robots = 3;
    c.prior_noise = noiseModel::Isotropic::Variance(6, 1e-12);
    auto odom_noise = noiseModel::Isotropic::Sigma(6, 0.05);
    auto loop_noise = noiseModel::Isotropic::Sigma(6, 0.05);

    std::mt19937 rng(seed);
    std::normal_distribution<double> rot_n(0.0, 0.08), trans_n(0.0, 0.15);

    std::vector<Values> gt(3);
    for (int r = 0; r < 3; ++r) {
        double base_yaw = r * 2.0 * M_PI / 3.0;
        double bx = 4.0 * std::cos(base_yaw), by = 4.0 * std::sin(base_yaw);
        for (int k = 0; k < 5; ++k) {
            Pose3 p(Rot3::RzRyRx(0.05 * r, -0.03 * r, base_yaw + 0.15 * k),
                    Point3(bx + 1.5 * k * std::cos(base_yaw + 0.5),
                           by + 1.5 * k * std::sin(base_yaw + 0.5),
                           0.2 * r + 0.1 * k));
            gt[r].insert(Symbol('a' + r, k), p);
        }
    }
    c.anchor_key = Symbol('a', 0);
    c.anchor_pose = gt[0].at<Pose3>(c.anchor_key);
    c.full_graph.add(PriorFactor<Pose3>(c.anchor_key, c.anchor_pose, c.prior_noise));

    for (int r = 0; r < 3; ++r) {
        c.subgraphs.push_back(boost::make_shared<NonlinearFactorGraph>());
        c.initials.push_back(boost::make_shared<Values>());
        Pose3 perturb = (r == 0) ? Pose3()
                                 : Pose3(Rot3::RzRyRx(rot_n(rng), rot_n(rng), rot_n(rng)),
                                         Point3(trans_n(rng), trans_n(rng), trans_n(rng)));
        for (int k = 0; k < 5; ++k) {
            Key key = Symbol('a' + r, k);
            c.initials[r]->insert(key, perturb * gt[r].at<Pose3>(key));
        }
        for (int k = 0; k < 4; ++k) {
            Key k0 = Symbol('a' + r, k), k1 = Symbol('a' + r, k + 1);
            Pose3 meas = gt[r].at<Pose3>(k0).between(gt[r].at<Pose3>(k1));
            auto f = boost::make_shared<BetweenFactor<Pose3>>(k0, k1, meas, odom_noise);
            c.subgraphs[r]->push_back(f);
            c.full_graph.push_back(f);
        }
    }
    std::vector<std::pair<Key, Key>> inter = {
        {Symbol('a', 4), Symbol('b', 0)},
        {Symbol('b', 4), Symbol('c', 0)},
        {Symbol('c', 4), Symbol('a', 0)},
    };
    for (auto e : inter) {
        int r0 = symbolChr(e.first) - 'a', r1 = symbolChr(e.second) - 'a';
        Pose3 noisy_delta(Rot3::RzRyRx(rot_n(rng), rot_n(rng), rot_n(rng)),
                          Point3(trans_n(rng), trans_n(rng), trans_n(rng)));
        Pose3 meas = gt[r0].at<Pose3>(e.first).between(gt[r1].at<Pose3>(e.second)) * noisy_delta;
        auto f = boost::make_shared<BetweenFactor<Pose3>>(e.first, e.second, meas, loop_noise);
        c.subgraphs[r0]->push_back(f);
        c.subgraphs[r1]->push_back(f);
        c.full_graph.push_back(f);
    }
    return c;
}

static GraphCase loadAuthorFixture() {
    GraphCase c;
    c.name = "Author fixture (0.g2o + 1.g2o, 64 poses)";
    c.num_robots = 2;
    c.prior_noise = noiseModel::Isotropic::Variance(6, 1e-12);
    auto g0 = readG2o("/tmp/dgs_fixture/0.g2o", true);
    auto g1 = readG2o("/tmp/dgs_fixture/1.g2o", true);
    c.subgraphs = {g0.first, g1.first};
    c.initials = {g0.second, g1.second};
    c.anchor_key = g0.second->keys().front();
    c.anchor_pose = g0.second->at<Pose3>(c.anchor_key);

    std::set<std::pair<Key, Key>> seen;
    c.full_graph.add(PriorFactor<Pose3>(c.anchor_key, c.anchor_pose, c.prior_noise));
    for (int r = 0; r < 2; ++r) {
        for (const auto& f : *c.subgraphs[r]) {
            auto bf = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(f);
            if (!bf) continue;
            auto edge = std::minmax(bf->keys()[0], bf->keys()[1]);
            if (seen.insert(edge).second) c.full_graph.push_back(f);
        }
    }
    return c;
}

static double chordalRotationObj(const NonlinearFactorGraph& full_graph, const Values& poses) {
    double obj = 0.0;
    for (const auto& f : full_graph) {
        auto bf = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(f);
        if (!bf) continue;
        Matrix3 R0 = poses.at<Pose3>(bf->keys()[0]).rotation().matrix();
        Matrix3 R1 = poses.at<Pose3>(bf->keys()[1]).rotation().matrix();
        Matrix3 Rij = bf->measured().rotation().matrix();
        obj += (R1 - R0 * Rij).squaredNorm();
    }
    return obj;
}

struct RefSolution {
    Values poses;
    double rot_obj = 0.0;
    double pose_obj = 0.0;
};

static RefSolution solveCentralizedChordal(const GraphCase& c) {
    auto full_init = boost::make_shared<Values>();
    for (int r = 0; r < c.num_robots; ++r) full_init->insert(*c.initials[r]);
    auto relabeled_g = boost::make_shared<NonlinearFactorGraph>();
    auto relabeled_v = boost::make_shared<Values>();
    std::map<Key, Key> to_single, from_single;
    uint64_t idx = 0;
    for (const auto& kv : *full_init) {
        Key sk = Symbol('a', idx++);
        to_single[kv.key] = sk;
        from_single[sk] = kv.key;
        relabeled_v->insert(sk, kv.value);
    }
    for (const auto& f : c.full_graph) {
        auto pf = boost::dynamic_pointer_cast<PriorFactor<Pose3>>(f);
        if (pf) relabeled_g->add(PriorFactor<Pose3>(to_single[pf->key()], pf->prior(), pf->noiseModel()));
        auto bf = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(f);
        if (bf) relabeled_g->add(BetweenFactor<Pose3>(to_single[bf->keys()[0]], to_single[bf->keys()[1]], bf->measured(), bf->noiseModel()));
    }
    distributed_mapper::DistributedMapper dm('a');
    dm.setUseBetweenNoiseFlag(false);
    dm.setUseLandmarksFlag(false);
    dm.setVerbosity(distributed_mapper::DistributedMapper::ERROR);
    dm.loadSubgraphAndCreateSubgraphEdge(std::make_pair(relabeled_g, relabeled_v));
    dm.estimateRotation();
    dm.updateRotation();
    dm.convertLinearizedRotationToPoses();
    dm.estimatePoses();
    dm.updatePoses();
    dm.retractPose3Global();
    Values out;
    for (const auto& kv : dm.currentEstimate()) out.insert(from_single[kv.key], kv.value);
    Pose3 delta = c.anchor_pose * out.at<Pose3>(c.anchor_key).inverse();
    RefSolution ref;
    for (const auto& kv : out) ref.poses.insert(kv.key, delta * out.at<Pose3>(kv.key));
    ref.rot_obj = chordalRotationObj(c.full_graph, ref.poses);
    ref.pose_obj = c.full_graph.error(ref.poses);
    return ref;
}

struct SimResult {
    int rot_iters = 0, pose_iters = 0;
    double rot_ms = 0.0, pose_ms = 0.0;
    double rot_obj = 0.0, pose_obj = 0.0;
    double rot_err = 0.0, trans_err = 0.0;
    std::string rot_reason, pose_reason;
};

static SimResult runDGS(const GraphCase& c, const RefSolution& ref,
                        bool float32_comm, double tol, int max_iter, bool old_freeze = false) {
    int n = c.num_robots;
    std::vector<boost::shared_ptr<distributed_mapper::DistributedMapper>> mappers(n);
    std::vector<OptimizationRound> rounds(n);
    std::vector<std::map<int, neighbor_estimateConstPtr>> rot_inbox(n), pose_inbox(n);
    std::vector<bool> rot_local_conv(n, false), pose_local_conv(n, false);
    std::vector<int> rot_iter(n, 0), pose_iter(n, 0);
    std::vector<Point3> anchor_offset(n, Point3::Zero());
    std::vector<std::map<int, Point3>> nbr_anchor_offset(n);

    std::vector<std::map<int, std::vector<Key>>> send_keys(n), recv_keys(n);
    std::vector<uint64_t> sep_keys;
    for (const auto& f : c.full_graph) {
        if (f->keys().size() != 2) continue;
        Key a = f->keys()[0], b = f->keys()[1];
        if (a > b) std::swap(a, b);
        int ra = OptimizationRound::robot(a), rb = OptimizationRound::robot(b);
        if (ra != rb) {
            sep_keys.push_back(a); sep_keys.push_back(b);
            send_keys[ra][rb].push_back(a); recv_keys[ra][rb].push_back(b);
            send_keys[rb][ra].push_back(b); recv_keys[rb][ra].push_back(a);
        }
    }
    for (int r = 0; r < n; ++r) {
        for (auto& kv : send_keys[r]) {
            std::sort(kv.second.begin(), kv.second.end());
            kv.second.erase(std::unique(kv.second.begin(), kv.second.end()), kv.second.end());
        }
        for (auto& kv : recv_keys[r]) {
            std::sort(kv.second.begin(), kv.second.end());
            kv.second.erase(std::unique(kv.second.begin(), kv.second.end()), kv.second.end());
        }
    }

    optimization_state commit;
    commit.round_id = 1;
    commit.coordinator = 0;
    for (int r = 0; r < n; ++r) commit.participants.push_back(r);
    commit.proposal_keys = sep_keys;
    commit.separator_keys = sep_keys;
    commit.order = OptimizationRound::connectedOrder(commit.participants, sep_keys);
    commit.committed = true;
    commit.state = 3;

    for (int r = 0; r < n; ++r) {
        rounds[r].begin(commit);
        rounds[r].status = commit;
        mappers[r].reset(new distributed_mapper::DistributedMapper(char('a' + r)));
        mappers[r]->setUseBetweenNoiseFlag(false);
        mappers[r]->setUseLandmarksFlag(false);
        mappers[r]->setVerbosity(distributed_mapper::DistributedMapper::ERROR);
        mappers[r]->setFlaggedInit(true);
        mappers[r]->setUpdateType(distributed_mapper::DistributedMapper::incUpdate);
        mappers[r]->setGamma(1.0);
        mappers[r]->loadSubgraphAndCreateSubgraphEdge(std::make_pair(c.subgraphs[r], c.initials[r]));
        if (r == 0) mappers[r]->addPrior(c.anchor_key, c.anchor_pose, c.prior_noise);
        mappers[r]->updateInitialized(false);
        mappers[r]->clearNeighboringRobotInit();
    }

    auto publish = [&](int r, bool rotation, std::vector<std::map<int, std::vector<double>>>& f64_cache) {
        for (int other = 0; other < n; ++other) if (other != r) {
            auto msg = boost::make_shared<neighbor_estimate>();
            msg->round_id = 1;
            msg->receiver_id = other;
            msg->iteration = rotation ? rot_iter[r] : pose_iter[r];
            msg->initialized = msg->iteration != 0;
            msg->local_converged = rotation ? rot_local_conv[r] : pose_local_conv[r];
            msg->estimation_done = rotation ? rounds[r].status.rotation_done : rounds[r].status.pose_done;
            for (int j = 0; j < 3; ++j) msg->anchor_offset.push_back(float(anchor_offset[r][j]));
            std::vector<double> raw;
            for (Key k : send_keys[r][other]) {
                msg->pose_id.push_back(Symbol(k).index());
                Vector v = rotation ? mappers[r]->linearizedRotationAt(k) : mappers[r]->linearizedPoses().at(k);
                for (int j = 0; j < v.size(); ++j) {
                    msg->estimate.push_back(float(v[j]));
                    raw.push_back(v[j]);
                }
            }
            if (!float32_comm) f64_cache[r][other] = std::move(raw);
            auto& inbox = rotation ? rot_inbox[other] : pose_inbox[other];
            rounds[other].retainEstimate(other, r, msg, inbox);
        }
    };

    auto apply_nbrs = [&](int r, bool rotation, const std::vector<std::map<int, std::vector<double>>>& f64_cache) {
        mappers[r]->clearNeighboringRobotInit();
        auto& inbox = rotation ? rot_inbox[r] : pose_inbox[r];
        const size_t w = rotation ? 9 : 6;
        for (int other = 0; other < n; ++other) if (other != r) {
            const auto& msg = *inbox.at(other);
            const auto& expected = recv_keys[r][other];
            for (size_t i = 0; i < expected.size(); ++i) {
                Vector v(w);
                for (size_t j = 0; j < w; ++j)
                    v[j] = float32_comm ? double(msg.estimate[w * i + j]) : f64_cache[other].at(r)[w * i + j];
                if (rotation) mappers[r]->updateNeighborLinearizedRotations(expected[i], v);
                else mappers[r]->updateNeighborLinearizedPoses(expected[i], v);
            }
            mappers[r]->updateNeighboringRobotInitialized(char('a' + other), msg.initialized);
            if (!rotation && msg.estimation_done)
                nbr_anchor_offset[r][other] = Point3(msg.anchor_offset[0], msg.anchor_offset[1], msg.anchor_offset[2]);
        }
    };

    auto run_phase = [&](bool rotation, std::string& reason_out) -> double {
        auto t0 = std::chrono::steady_clock::now();
        std::vector<std::map<int, std::vector<double>>> f64_cache(n);
        for (int r = 0; r < n; ++r) publish(r, rotation, f64_cache);
        bool any_converged = false;
        for (int ticks = 0; ticks < max_iter * n * 4; ++ticks) {
            bool all_done = true;
            for (int r : commit.order) {
                auto& done = rotation ? rounds[r].status.rotation_done : rounds[r].status.pose_done;
                if (done) continue;
                all_done = false;
                int& iter = rotation ? rot_iter[r] : pose_iter[r];
                auto& inbox = rotation ? rot_inbox[r] : pose_inbox[r];
                bool lconv = rotation ? rot_local_conv[r] : pose_local_conv[r];
                if (!old_freeze && iter > 0 && rounds[r].allConvergedAt(r, iter, lconv, max_iter, inbox)) {
                    done = true;
                    any_converged = true;
                    publish(r, rotation, f64_cache);
                    continue;
                }
                if (old_freeze) {
                    bool ready = true;
                    bool before = true;
                    for (int id : commit.order) {
                        if (id == r) { before = false; continue; }
                        auto it = inbox.find(id);
                        uint32_t needed = uint32_t(iter) + (before ? 1u : 0u);
                        if (it == inbox.end() || (!it->second->estimation_done && it->second->iteration < needed)) {
                            ready = false; break;
                        }
                    }
                    if (!ready) continue;
                } else {
                    if (!rounds[r].stepReady(r, iter, inbox)) continue;
                }
                apply_nbrs(r, rotation, f64_cache);
                if (rotation) { mappers[r]->estimateRotation(); mappers[r]->updateRotation(); }
                else { mappers[r]->estimatePoses(); mappers[r]->updatePoses(); }
                mappers[r]->updateInitialized(true);
                ++iter;
                if (old_freeze) {
                    bool lc = mappers[r]->latestChange() <= tol;
                    if (lc) any_converged = true;
                    done = lc || (iter >= max_iter);
                } else {
                    int min_s = std::min(2, max_iter);
                    bool lc = (iter >= min_s) && (mappers[r]->latestChange() <= tol);
                    if (rotation) rot_local_conv[r] = lc; else pose_local_conv[r] = lc;
                    bool gconv = rounds[r].allConvergedAt(r, iter, lc, max_iter, inbox);
                    if (gconv) any_converged = true;
                    done = gconv || (iter >= max_iter);
                }
                if (!rotation) {
                    Key first = mappers[r]->currentEstimate().keys().front();
                    Point3 ap = c.initials[r]->at<Pose3>(first).translation();
                    anchor_offset[r] = ap - (mappers[r]->currentEstimate().at<Pose3>(first).translation() +
                                             Point3(mappers[r]->linearizedPoses().at(first).tail(3)));
                }
                publish(r, rotation, f64_cache);
            }
            if (all_done) break;
        }
        for (int r = 0; r < n; ++r) apply_nbrs(r, rotation, f64_cache);
        auto t1 = std::chrono::steady_clock::now();
        reason_out = any_converged ? "converged" : "max_iter";
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    };

    SimResult res;
    res.rot_ms = run_phase(true, res.rot_reason);

    // Measure rotation-stage objective and error right after rotation projection
    Values rot_poses;
    for (int r = 0; r < n; ++r) {
        mappers[r]->convertLinearizedRotationToPoses();
        rot_poses.insert(mappers[r]->currentEstimate());
        Values nbrs = mappers[r]->neighbors();
        for (const auto& kv : nbrs) {
            VectorValues vv;
            vv.insert(kv.key, mappers[r]->neighborsLinearizedRotationsAt(kv.key));
            Values nr = InitializePose3::normalizeRelaxedRotations(vv);
            Values np = distributed_mapper::evaluation_utils::pose3WithZeroTranslation(nr);
            mappers[r]->updateNeighbor(kv.key, np.at<Pose3>(kv.key));
        }
        mappers[r]->updateInitialized(false);
        mappers[r]->clearNeighboringRobotInit();
        mappers[r]->resetLatestChange();
    }
    res.rot_obj = chordalRotationObj(c.full_graph, rot_poses);

    res.pose_ms = run_phase(false, res.pose_reason);

    Values merged;
    for (int r = 0; r < n; ++r) {
        mappers[r]->retractPose3GlobalWithOffset(r == 0 ? anchor_offset[r] : nbr_anchor_offset[r][0]);
        merged.insert(mappers[r]->currentEstimate());
    }
    Pose3 delta = c.anchor_pose * merged.at<Pose3>(c.anchor_key).inverse();
    Values aligned;
    for (const auto& kv : merged) aligned.insert(kv.key, delta * merged.at<Pose3>(kv.key));

    res.rot_iters = rot_iter[0];
    res.pose_iters = pose_iter[0];
    res.pose_obj = c.full_graph.error(aligned);
    double r_num = 0, r_den = 0, t_num = 0, t_den = 0;
    for (const auto& kv : ref.poses) {
        Pose3 p_ref = ref.poses.at<Pose3>(kv.key), p_est = aligned.at<Pose3>(kv.key);
        Matrix3 dR = p_est.rotation().matrix() - p_ref.rotation().matrix();
        Vector3 dt = p_est.translation() - p_ref.translation();
        r_num += dR.squaredNorm();
        r_den += p_ref.rotation().matrix().squaredNorm();
        t_num += dt.squaredNorm();
        t_den += p_ref.translation().squaredNorm();
    }
    res.rot_err = std::sqrt(r_num) / std::max(1.0, std::sqrt(r_den));
    res.trans_err = std::sqrt(t_num) / std::max(1.0, std::sqrt(t_den));
    return res;
}

int main() {
    std::vector<GraphCase> cases = {
        makeFreezeCounterexample(),
        makeNoisyCycle3(2),
        makeNoisyCycle3(7),
        makeNoisyCycle3(19),
    };
    if (std::ifstream("/tmp/dgs_fixture/0.g2o").good() && std::ifstream("/tmp/dgs_fixture/1.g2o").good()) {
        cases.push_back(loadAuthorFixture());
    }

    const std::vector<double> tols = {0.1, 1e-3, 1e-4, 1e-5};
    const std::vector<int> max_iters = {30, 100, 200};

    for (const auto& c : cases) {
        RefSolution ref = solveCentralizedChordal(c);
        std::cout << "\n=== " << c.name << " | ref rot_obj=" << std::scientific << std::setprecision(3)
                  << ref.rot_obj << " pose_obj=" << ref.pose_obj << " ===\n";
        std::cout << "  tol      max_it | rot_it rot_ms  rot_obj   rot_err   rot_stop  | pose_it pose_ms pose_obj  trans_err pose_stop\n";
        for (double tol : tols) {
            for (int mi : max_iters) {
                SimResult s = runDGS(c, ref, true, tol, mi);
                std::cout << "  " << std::left << std::setw(8) << std::scientific << std::setprecision(0) << tol
                          << " " << std::right << std::setw(6) << mi << " | "
                          << std::setw(6) << s.rot_iters << " "
                          << std::setw(6) << std::fixed << std::setprecision(1) << s.rot_ms << " "
                          << std::setw(9) << std::scientific << std::setprecision(3) << s.rot_obj << " "
                          << std::setw(9) << s.rot_err << " "
                          << std::left << std::setw(9) << s.rot_reason << " | "
                          << std::right << std::setw(7) << s.pose_iters << " "
                          << std::setw(7) << std::fixed << std::setprecision(1) << s.pose_ms << " "
                          << std::setw(9) << std::scientific << std::setprecision(3) << s.pose_obj << " "
                          << std::setw(9) << s.trans_err << " "
                          << std::left << std::setw(9) << s.pose_reason << "\n";
            }
        }
    }
    std::cout << "\n=== Policy & Precision Comparison (tol=1e-4, max_iter=100) ===\n";
    std::cout << "  case                                      | policy        | rot_it rot_err   | pose_it trans_err\n";
    for (const auto& c : cases) {
        RefSolution ref = solveCentralizedChordal(c);
        struct Cfg { const char* label; bool f32; bool old_f; };
        for (const auto& cfg : std::vector<Cfg>{{"OLD_FREEZE_F32", true, true},
                                               {"NEW_COORD_F32",  true, false},
                                               {"NEW_COORD_F64",  false, false}}) {
            SimResult s = runDGS(c, ref, cfg.f32, 1e-4, 100, cfg.old_f);
            std::cout << "  " << std::left << std::setw(41) << c.name << " | "
                      << std::setw(13) << cfg.label << " | "
                      << std::right << std::setw(6) << s.rot_iters << " "
                      << std::scientific << std::setprecision(3) << s.rot_err << " | "
                      << std::setw(7) << s.pose_iters << " "
                      << s.trans_err << "\n";
        }
    }
    return 0;
}
