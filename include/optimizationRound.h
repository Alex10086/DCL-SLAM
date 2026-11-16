#ifndef DCL_OPTIMIZATION_ROUND_H
#define DCL_OPTIMIZATION_ROUND_H

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
#include "dcl_slam/optimization_state.h"
#include "dcl_slam/neighbor_estimate.h"

namespace dcl_slam {
// Transport-independent barriers shared by the backend and deterministic tests.
// A round never infers readiness from an unrelated robot's numeric state.
class OptimizationRound {
public:
    optimization_state status;
    std::map<int, optimization_stateConstPtr> peers;
    std::vector<uint64_t> proposal_edges;

    static int robot(uint64_t key) { return int(key >> 56) - int('a'); }
    static bool contains(const std::vector<int32_t>& ids, int id) {
        return std::find(ids.begin(), ids.end(), id) != ids.end();
    }
    static bool valid(const optimization_state& s, int count) {
        if (!s.round_id || s.participants.size() < 2 || s.proposal_keys.empty() ||
            s.proposal_keys.size() % 2 || s.separator_keys.size() % 2) return false;
        if (!std::is_sorted(s.participants.begin(), s.participants.end()) ||
            std::adjacent_find(s.participants.begin(), s.participants.end()) != s.participants.end() ||
            s.participants.front() < 0 || s.participants.back() >= count ||
            s.coordinator != s.participants.front()) return false;
        for (const auto* keys : {&s.proposal_keys, &s.separator_keys})
            for (size_t i = 0; i < keys->size(); i += 2) {
                auto a = (*keys)[i], b = (*keys)[i+1];
                if (a >= b || robot(a) == robot(b) || !contains(s.participants, robot(a)) ||
                    !contains(s.participants, robot(b))) return false;
                if (i && std::make_pair((*keys)[i-2], (*keys)[i-1]) >= std::make_pair(a,b)) return false;
            }
        if (s.committed) {
            if (s.order.size() != s.participants.size() || s.order.front() != s.coordinator) return false;
            for (int id : s.participants)
                if (std::count(s.order.begin(),s.order.end(),id) != 1) return false;
        }
        return s.state >= 0 && s.state <= 7;
    }
    void begin(const optimization_state& proposal) {
        status = proposal;
        status.header.seq = 0;
        status.state = 1;
        status.committed = status.aborted = status.rotation_done = status.pose_done = false;
        proposal_edges = proposal.proposal_keys;
        status.order.clear();
        peers.clear();
    }
    bool same(const optimization_state& s) const {
        return s.round_id == status.round_id && s.coordinator == status.coordinator &&
               s.participants == status.participants && s.proposal_keys == status.proposal_keys;
    }
    bool matchingCommit(const optimization_state& s) const {
        return same(s) && s.committed && s.order == status.order &&
               s.separator_keys == status.separator_keys && !s.aborted;
    }
    bool receive(int sender, const optimization_stateConstPtr& s) {
        if (!same(*s) || !contains(status.participants, sender)) return false;
        const auto old = peers.find(sender);
        if (old != peers.end() && s->header.seq <= old->second->header.seq) return false;
        peers[sender] = s;
        if (s->aborted) status.aborted = true;
        return true;
    }
    bool allPrepared(int self) const {
        if (status.state < 2 || status.aborted) return false;
        for (int id : status.participants) if (id != self) {
            auto p = peers.find(id);
            if (p == peers.end() || p->second->state < 2 || p->second->aborted) return false;
        }
        return true;
    }
    // An edge is usable only if both endpoints retained it after local PCM.
    std::vector<uint64_t> agreedEdges(int self) const {
        std::vector<uint64_t> edges;
        edges.reserve(proposal_edges.size());
        for (size_t i = 0; i < proposal_edges.size(); i += 2) {
            const auto a = proposal_edges[i], b = proposal_edges[i+1];
            bool agreed = true;
            for (int endpoint : {robot(a), robot(b)}) {
                const auto& keys = endpoint == self ? status.separator_keys : peers.at(endpoint)->separator_keys;
                bool found = false;
                for (size_t j = 0; j < keys.size(); j += 2)
                    found |= keys[j] == a && keys[j+1] == b;
                agreed &= found;
            }
            if (agreed) { edges.push_back(a); edges.push_back(b); }
        }
        return edges;
    }
    // Original largest-edge-first order, with a deterministic lower-ID tie break.
    static std::vector<int32_t> connectedOrder(const std::vector<int32_t>& members,
                                               const std::vector<uint64_t>& edges) {
        if (members.empty()) return {};
        std::vector<int32_t> order;
        order.reserve(members.size());
        order.push_back(members.front());
        while (order.size() < members.size()) {
            int best = -1, score = 0;
            for (int id : members) if (!contains(order, id)) {
                int degree = 0;
                for (size_t i = 0; i < edges.size(); i += 2) {
                    int a = robot(edges[i]), b = robot(edges[i+1]);
                    degree += (a == id && contains(order,b)) || (b == id && contains(order,a));
                }
                if (degree > score) { best = id; score = degree; }
            }
            if (best < 0) return {};
            order.push_back(best);
        }
        return order;
    }
    bool allCommitted(int self) const {
        if (!status.committed || status.aborted) return false;
        for (int id : status.participants) if (id != self) {
            auto p = peers.find(id);
            if (p == peers.end() || !matchingCommit(*p->second)) return false;
        }
        return true;
    }
    bool phaseDone(int self, bool rotation) const {
        if (!(rotation ? status.rotation_done : status.pose_done) || status.aborted) return false;
        for (int id : status.participants) if (id != self) {
            auto p = peers.find(id);
            if (p == peers.end() || !matchingCommit(*p->second) ||
                !(rotation ? p->second->rotation_done : p->second->pose_done)) return false;
        }
        return true;
    }
    bool allEnded(int self) const {
        if (status.aborted || !status.committed) return false;
        for (int id : status.participants) if (id != self) {
            auto p = peers.find(id);
            if (p == peers.end() || !matchingCommit(*p->second) ||
                (p->second->state < 6 && !(p->second->state == 0 && p->second->pose_done))) return false;
        }
        return true;
    }
    // Ordered incremental (Gauss-Seidel), not simultaneous/Jacobi updates.
    bool stepReady(int self, uint32_t iteration,
                   const std::map<int, neighbor_estimateConstPtr>& inbox) const {
        bool before = true;
        for (int id : status.order) {
            if (id == self) { before = false; continue; }
            auto p = inbox.find(id);
            if (p == inbox.end() || p->second->round_id != status.round_id) return false;
            const uint32_t needed = iteration + (before ? 1 : 0);
            if (!p->second->estimation_done && p->second->iteration < needed) return false;
        }
        return status.committed && !status.aborted;
    }
    bool acceptsEstimate(int self, int sender, const neighbor_estimate& msg) const {
        return status.round_id && msg.round_id == status.round_id && !status.aborted &&
               msg.receiver_id == self && contains(status.participants,sender);
    }
    bool retainEstimate(int self, int sender, const neighbor_estimateConstPtr& msg,
                        std::map<int, neighbor_estimateConstPtr>& inbox) const {
        if (!acceptsEstimate(self,sender,*msg)) return false;
        auto old = inbox.find(sender);
        if (old != inbox.end() && old->second->iteration >= msg->iteration) return false;
        inbox[sender] = msg;
        return true;
    }
    bool finalEstimatesReady(int self, const std::map<int, neighbor_estimateConstPtr>& inbox) const {
        for (int id : status.participants) if (id != self) {
            auto p = inbox.find(id);
            if (p == inbox.end() || !acceptsEstimate(self,id,*p->second) ||
                !p->second->estimation_done) return false;
        }
        return !status.aborted;
    }
};
}
#endif
