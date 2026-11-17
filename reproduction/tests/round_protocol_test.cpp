#include "optimizationRound.h"
#include <boost/make_shared.hpp>
#include <array>
#include <functional>
#include <iostream>
#include <stdexcept>

using dcl_slam::OptimizationRound;
using dcl_slam::optimization_state;
using dcl_slam::neighbor_estimate;
using dcl_slam::neighbor_estimateConstPtr;

static void require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}
static uint64_t key(char robot, int index = 0) {
    return (uint64_t(robot) << 56) | uint64_t(index);
}
static optimization_state proposal(uint64_t id = 700) {
    optimization_state s;
    s.round_id = id;
    s.coordinator = 0;
    s.participants = {0,1,2};
    s.state = 1;
    s.separator_keys = {key('a'),key('b'),key('a'),key('c'),key('b'),key('c')};
    s.proposal_keys = s.separator_keys;
    return s;
}
struct Team {
    std::array<OptimizationRound,3> nodes;
    Team() { for (auto& node : nodes) node.begin(proposal()); }
    void ready(int id) {
        auto& node = nodes[id];
        node.status.state = 2;
        node.status.separator_keys.clear();
        for (size_t i = 0; i < node.proposal_edges.size(); i += 2) {
            auto a = node.proposal_edges[i], b = node.proposal_edges[i+1];
            if (OptimizationRound::robot(a) == id || OptimizationRound::robot(b) == id) {
                node.status.separator_keys.push_back(a);
                node.status.separator_keys.push_back(b);
            }
        }
    }
    void send(int sender) {
        auto& status = nodes[sender].status;
        ++status.header.seq;
        for (int receiver = 0; receiver < 3; ++receiver) if (receiver != sender)
            nodes[receiver].receive(sender, dcl_slam::optimization_stateConstPtr(new optimization_state(status)));
    }
    void allReady() { for (int id = 0; id < 3; ++id) ready(id); for (int id = 0; id < 3; ++id) send(id); }
    void commit() {
        allReady();
        auto edges = nodes[0].agreedEdges(0);
        auto order = OptimizationRound::connectedOrder(nodes[0].status.participants,edges);
        require(!order.empty(), "connected PCM graph must yield rooted order");
        for (auto& node : nodes) {
            node.status.separator_keys = edges;
            node.status.order = order;
            node.status.committed = true;
        }
        for (int id = 0; id < 3; ++id) send(id);
    }
};
static neighbor_estimateConstPtr packet(uint64_t round, int receiver, uint32_t iteration,
                                       bool done = false, bool local_converged = false) {
    auto p = boost::make_shared<neighbor_estimate>();
    p->round_id = round;
    p->receiver_id = receiver;
    p->iteration = iteration;
    p->estimation_done = done;
    p->local_converged = local_converged;
    p->initialized = iteration != 0;
    p->anchor_offset = {1,2,3};
    return p;
}

static void staggered_prepare_and_commit() {
    Team t;
    t.ready(0); t.ready(2); t.send(0); t.send(2);
    require(!t.nodes[0].allPrepared(0), "A/C must not start without delayed B READY");
    require(!t.nodes[2].allPrepared(2), "C must not infer B readiness from A state");
    // A late subscriber can adopt the leader's READY without losing nonleader edges.
    OptimizationRound late;
    late.begin(t.nodes[0].status);
    require(late.proposal_edges == proposal().proposal_keys, "late READY adoption lost B-C proposal edge");
    t.ready(1); t.send(1);
    require(t.nodes[0].allPrepared(0), "late B READY must unblock preparation");
    auto edges = t.nodes[0].agreedEdges(0);
    t.nodes[0].status.separator_keys = edges;
    t.nodes[0].status.order = OptimizationRound::connectedOrder({0,1,2},edges);
    t.nodes[0].status.committed = true;
    require(!t.nodes[0].allCommitted(0), "leader must wait for every COMMIT acknowledgement");
    t.send(0);
    for (int id : {1,2}) {
        t.nodes[id].status.separator_keys = edges;
        t.nodes[id].status.order = t.nodes[0].status.order;
        t.nodes[id].status.committed = true;
        t.send(id);
    }
    require(t.nodes[0].allCommitted(0), "coherent commit acknowledgements must release leader");
    t.nodes[1].status.order = {0,2,1}; t.send(1);
    require(!t.nodes[0].allCommitted(0), "mismatched optimization order must not count as ACK");
}
static void conservative_pcm_consensus() {
    Team t; t.allReady();
    t.nodes[1].status.separator_keys = {key('a'),key('b')}; t.send(1);
    auto agreed = t.nodes[0].agreedEdges(0);
    require(agreed == std::vector<uint64_t>({key('a'),key('b'),key('a'),key('c')}),
            "one-sided PCM-rejected B-C edge must not enter commit");
    require(OptimizationRound::connectedOrder({0,1,2},agreed) == std::vector<int32_t>({0,1,2}),
            "consensus must retain a connected rooted order");
    t.nodes[0].status.separator_keys = {key('a'),key('b')};
    t.nodes[2].status.separator_keys.clear(); t.send(2);
    agreed = t.nodes[0].agreedEdges(0);
    require(OptimizationRound::connectedOrder({0,1,2},agreed).empty(),
            "PCM-disconnected C must abort rather than silently shrink team");
}
static void ordered_sweeps_and_early_packets() {
    Team t; t.commit();
    auto& b = t.nodes[1];
    std::map<int,neighbor_estimateConstPtr> inbox;
    require(b.retainEstimate(1,0,packet(700,1,0),inbox), "initial A packet must be retained during Initialization");
    b.retainEstimate(1,2,packet(700,1,0),inbox);
    require(!b.stepReady(1,0,inbox), "B must not run Jacobi step before preceding A update");
    b.retainEstimate(1,0,packet(700,1,1),inbox);
    require(b.stepReady(1,0,inbox), "buffered early A update must release delayed B");
    require(!b.stepReady(1,1,inbox), "duplicate snapshots must not run a second sweep");
    require(!b.retainEstimate(1,0,packet(699,1,90,true),inbox), "old round estimates must not replace current estimates");
    require(!b.retainEstimate(1,0,packet(700,2,90,true),inbox), "another receiver's packet must not be consumed");
    require(!b.retainEstimate(1,0,packet(700,1,0),inbox), "out-of-order iteration must not roll cache backward");
    require(inbox.at(0)->iteration == 1, "early update was lost after stale packet");
    require(!b.allConvergedAt(1,1,true,30,inbox), "single sweep must not trigger global convergence");
    b.retainEstimate(1,0,packet(700,1,2,false,true),inbox);
    b.retainEstimate(1,2,packet(700,1,2,false,false),inbox);
    require(!b.allConvergedAt(1,2,true,30,inbox), "unconverged C must keep team running sweep 3");
    require(b.retainEstimate(1,2,packet(700,1,2,true,true),inbox),
            "same-iteration transition to estimation_done must be retained");
    require(b.allConvergedAt(1,2,true,30,inbox), "coherent converged team at sweep 2 must stop");
    require(!b.retainEstimate(1,2,packet(700,1,2,false,true),inbox),
            "estimation_done must not regress at same iteration");
    b.retainEstimate(1,0,packet(700,1,30,true,false),inbox);
    b.retainEstimate(1,2,packet(700,1,30,true,true),inbox);
    require(!b.allConvergedAt(1,30,true,30,inbox),
            "max_iter estimation_done without local_converged must not count as convergence");
}
static void final_payload_and_phase_barriers() {
    Team t; t.commit();
    for (int id = 0; id < 3; ++id) { t.nodes[id].status.rotation_done = true; t.send(id); }
    auto& a = t.nodes[0];
    require(a.phaseDone(0,true), "all frozen rotation statuses should satisfy status barrier");
    std::map<int,neighbor_estimateConstPtr> inbox;
    a.retainEstimate(0,1,packet(700,0,1,true),inbox);
    a.retainEstimate(0,2,packet(700,0,0),inbox);
    require(!a.finalEstimatesReady(0,inbox), "status done must not substitute for C final rotation payload");
    a.retainEstimate(0,2,packet(700,0,1,true),inbox);
    require(a.finalEstimatesReady(0,inbox), "all final rotation payloads should release pose initialization");
    require(!a.phaseDone(0,false), "rotation completion must not count as pose completion");
    for (int id = 0; id < 3; ++id) {
        t.nodes[id].status.pose_done = true; t.nodes[id].status.state = 7; t.send(id);
    }
    require(a.phaseDone(0,false) && a.allEnded(0), "joint finished round must release ending barrier");
    t.nodes[1].status.state = 0; t.send(1);
    require(a.allEnded(0), "peer already Idle after completion must not strand last PostEnd node");
}
static void stale_states_and_abort_are_terminal() {
    Team t; t.commit();
    auto stale = boost::make_shared<optimization_state>(t.nodes[1].status);
    stale->round_id = 699; stale->state = 6; stale->header.seq = 1000;
    require(!t.nodes[0].receive(1,stale), "historical End must not satisfy current round");
    t.nodes[1].status.aborted = true; t.send(1);
    require(t.nodes[0].status.aborted, "peer abort must propagate to round");
    t.nodes[1].status.aborted = false; t.nodes[1].status.state = 6; t.send(1);
    require(t.nodes[0].status.aborted && !t.nodes[0].allCommitted(0) && !t.nodes[0].allEnded(0),
            "later normal status must not resurrect aborted round");
    t.nodes[0].begin(proposal(800));
    require(!t.nodes[0].status.aborted && !t.nodes[0].receive(1,stale),
            "new round must reset abort but continue rejecting old evidence");
}
static void malformed_membership_is_rejected() {
    auto s = proposal(); require(OptimizationRound::valid(s,3), "valid proposal rejected");
    s.participants = {0,1,1}; require(!OptimizationRound::valid(s,3), "duplicate participant accepted");
    s = proposal(); s.separator_keys.push_back(key('a')); require(!OptimizationRound::valid(s,3), "partial edge accepted");
    s = proposal(); s.committed = true; s.order = {1,0,2}; require(!OptimizationRound::valid(s,3), "non-rooted order accepted");
}
int main() {
    const std::pair<const char*,std::function<void()>> tests[] = {
        {"staggered_prepare_and_commit",staggered_prepare_and_commit},
        {"conservative_pcm_consensus",conservative_pcm_consensus},
        {"ordered_sweeps_and_early_packets",ordered_sweeps_and_early_packets},
        {"final_payload_and_phase_barriers",final_payload_and_phase_barriers},
        {"stale_states_and_abort_are_terminal",stale_states_and_abort_are_terminal},
        {"malformed_membership_is_rejected",malformed_membership_is_rejected}};
    try {
        for (const auto& test : tests) { test.second(); std::cout << "PASS " << test.first << '\n'; }
    } catch (const std::exception& e) { std::cerr << "FAIL " << e.what() << '\n'; return 1; }
    return 0;
}
