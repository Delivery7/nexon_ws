// dstar_lite_core.cpp
// Simple, reasonably efficient D* Lite core implementation in C++
// Exposes minimal API used by planner above.

#include <vector>
#include <set>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <utility>

#ifndef DSTAR_LITE_CORE_HPP_
#define DSTAR_LITE_CORE_HPP_
namespace dstar {
using Cell = std::pair<int,int>;
const double INF = std::numeric_limits<double>::infinity();

struct Key {
  double k1;
  double k2;
  bool operator<(Key const & o) const {
    if (k1 == o.k1) return k2 < o.k2;
    return k1 < o.k1;
  }
};

class DStarLiteCore {
public:
  DStarLiteCore() = delete;
  DStarLiteCore(int width, int height, Cell start, Cell goal, double obstacle_cost = 1e9);
  ~DStarLiteCore() = default;

  void computeShortestPath(size_t max_iters = 1000000);
  std::vector<Cell> getPath() const;

  void setObstacle(const Cell & c);
  void clearObstacle(const Cell & c);
  void moveStart(const Cell & new_start);

private:
  int width_;
  int height_;
  int size_;
  Cell start_;
  Cell goal_;
  double obstacle_cost_;

  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<double> cost_; // cost of entering cell

  double km_;
  Cell last_;

  // Open list: map from key to node index; use set of pair<Key, idx>
  typedef std::pair<Key,int> KeyIndex;
  std::set<KeyIndex> U_;
  std::vector<bool> in_queue_;

  inline int idx(int x, int y) const { return y*width_ + x; }
  inline bool valid(int x, int y) const { return x>=0 && y>=0 && x<width_ && y<height_; }
  inline double heuristic(const Cell & a, const Cell & b) const {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
  }
  Key calculateKey(const Cell & u) const;
  std::vector<Cell> neighbours(const Cell & u) const;
  double costBetween(const Cell & a, const Cell & b) const;
  void updateVertex(const Cell & u);
  void insertOrUpdateQueue(int index, const Key & k);
  void removeFromQueue(int index);
  Key topKey() const;
};

} // namespace dstar
#endif // DSTAR_LITE_CORE_HPP_

// ================= Implementation =================

namespace dstar {

DStarLiteCore::DStarLiteCore(int width, int height, Cell start, Cell goal, double obstacle_cost)
: width_(width), height_(height), start_(start), goal_(goal), obstacle_cost_(obstacle_cost)
{
  size_ = width_ * height_;
  g_.assign(size_, INF);
  rhs_.assign(size_, INF);
  cost_.assign(size_, 1.0);
  in_queue_.assign(size_, false);
  km_ = 0.0;
  last_ = start_;

  // initialize goal rhs = 0 and push to U
  rhs_[ idx(goal_.first, goal_.second) ] = 0.0;
  Key k = calculateKey(goal_);
  insertOrUpdateQueue(idx(goal_.first, goal_.second), k);
}

Key DStarLiteCore::calculateKey(const Cell & u) const {
  int id = idx(u.first, u.second);
  double g_rhs = std::min(g_[id], rhs_[id]);
  Key k;
  k.k1 = g_rhs + heuristic(start_, u) + km_;
  k.k2 = g_rhs;
  return k;
}

std::vector<Cell> DStarLiteCore::neighbours(const Cell & u) const {
  std::vector<Cell> out;
  const int dx[4] = {1,-1,0,0};
  const int dy[4] = {0,0,1,-1};
  for (int i=0;i<4;++i){
    int nx = u.first + dx[i];
    int ny = u.second + dy[i];
    if (valid(nx, ny)) out.emplace_back(nx, ny);
  }
  return out;
}

double DStarLiteCore::costBetween(const Cell & a, const Cell & b) const {
  int idb = idx(b.first, b.second);
  return cost_[idb];
}

void DStarLiteCore::insertOrUpdateQueue(int index, const Key & k) {
  // remove old if exists
  if (in_queue_[index]) {
    // find and remove existing key pair
    // linear search in set is expensive; instead erase by searching for any KeyIndex with same idx
    for (auto it = U_.begin(); it != U_.end(); ++it) {
      if (it->second == index) {
        U_.erase(it);
        break;
      }
    }
  }
  U_.insert(std::make_pair(k, index));
  in_queue_[index] = true;
}

void DStarLiteCore::removeFromQueue(int index) {
  if (!in_queue_[index]) return;
  for (auto it = U_.begin(); it != U_.end(); ++it) {
    if (it->second == index) {
      U_.erase(it);
      break;
    }
  }
  in_queue_[index] = false;
}

Key DStarLiteCore::topKey() const {
  if (U_.empty()) return Key{INF, INF};
  return U_.begin()->first;
}

void DStarLiteCore::updateVertex(const Cell & u) {
  int uid = idx(u.first, u.second);
  if (!(u == goal_)) {
    double min_rhs = INF;
    auto nb = neighbours(u);
    for (const auto & s : nb) {
      int sid = idx(s.first, s.second);
      double val = g_[sid] + costBetween(u, s);
      if (val < min_rhs) min_rhs = val;
    }
    rhs_[uid] = min_rhs;
  }
  // update queue
  if (in_queue_[uid]) removeFromQueue(uid);
  if (g_[uid] != rhs_[uid]) {
    Key k = calculateKey(u);
    insertOrUpdateQueue(uid, k);
  }
}

void DStarLiteCore::computeShortestPath(size_t max_iters) {
  size_t iters = 0;
  Key k_start = calculateKey(start_);
  while ( (topKey() < k_start) || (rhs_[ idx(start_.first, start_.second) ] != g_[ idx(start_.first, start_.second) ]) ) {
    if (++iters > max_iters) {
      throw std::runtime_error("D* Lite exceeded max iterations");
    }
    if (U_.empty()) break;
    auto top = *U_.begin();
    Key k_old = top.first;
    int uidx = top.second;
    Cell u = { uidx % width_, uidx / width_ };
    removeFromQueue(uidx);
    Key k_new = calculateKey(u);
    if (k_old < k_new) {
      insertOrUpdateQueue(uidx, k_new);
    } else if (g_[uidx] > rhs_[uidx]) {
      g_[uidx] = rhs_[uidx];
      auto nb = neighbours(u);
      for (auto & s : nb) updateVertex(s);
    } else {
      double g_old = g_[uidx];
      g_[uidx] = INF;
      auto nb = neighbours(u);
      updateVertex(u);
      for (auto & s : nb) updateVertex(s);
    }
    k_start = calculateKey(start_);
  }
}

std::vector<Cell> DStarLiteCore::getPath() const {
  std::vector<Cell> path;
  Cell current = start_;
  int steps = 0;
  if (g_[ idx(current.first, current.second) ] == INF && rhs_[ idx(current.first, current.second) ] != 0.0) {
    return {}; // no path
  }
  while (!(current == goal_) && steps < width_*height_ + 5) {
    path.push_back(current);
    auto nb = neighbours(current);
    double min_val = INF;
    Cell best = {-1,-1};
    for (auto & s : nb) {
      double val = g_[ idx(s.first, s.second) ] + costBetween(current, s);
      if (val < min_val) {
        min_val = val;
        best = s;
      }
    }
    if (best.first == -1) return {}; // blocked
    current = best;
    ++steps;
  }
  path.push_back(goal_);
  return path;
}

void DStarLiteCore::setObstacle(const Cell & c) {
  if (!valid(c.first, c.second)) return;
  cost_[ idx(c.first, c.second) ] = obstacle_cost_;
  // update vertex and its neighbors
  updateVertex(c);
  auto nb = neighbours(c);
  for (auto & s : nb) updateVertex(s);
}

void DStarLiteCore::clearObstacle(const Cell & c) {
  if (!valid(c.first, c.second)) return;
  cost_[ idx(c.first, c.second) ] = 1.0;
  updateVertex(c);
  auto nb = neighbours(c);
  for (auto & s : nb) updateVertex(s);
}

void DStarLiteCore::moveStart(const Cell & new_start) {
  km_ += heuristic(last_, new_start);
  start_ = new_start;
  last_ = new_start;
  updateVertex(start_);
}

} // namespace dstar
