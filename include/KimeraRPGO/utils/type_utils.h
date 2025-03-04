// Authors: Yun Chang
#pragma once 

#include <functional>
#include <memory>
#include <utility>

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "KimeraRPGO/logger.h"

namespace KimeraRPGO {

struct Measurements {
  gtsam::NonlinearFactorGraph factors;
  gtsam::NonlinearFactorGraph consistent_factors;
  gtsam::Matrix adj_matrix;
  gtsam::Matrix dist_matrix;

  Measurements(
      gtsam::NonlinearFactorGraph new_factors = gtsam::NonlinearFactorGraph())
      : factors(new_factors), consistent_factors(new_factors) {
    if (new_factors.size() > 1) {
      log<WARNING>(
          "Unexpected behavior: initializing Measurement struct with more than "
          "one factor.");
    }
    adj_matrix = Eigen::MatrixXd::Zero(1, 1);
    dist_matrix = Eigen::MatrixXd::Zero(1, 1);
  }
};

// struct storing the involved parties (ex robot a and robot b)
struct ObservationId {//ObservationId: 机器人a与机器人a，机器人a与机器人b，机器人a与机器人c...
  char id1;
  char id2;

  ObservationId(char first, char second) {
    id1 = first;
    id2 = second;
  }

  bool operator==(const ObservationId& other) const {
    if (id1 == other.id1 && id2 == other.id2) return true;
    if (id2 == other.id1 && id1 == other.id2) return true;
    return false;
  }
};

// Add compatibility for c++11's lack of make_unique.
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace KimeraRPGO


//std::hash的特化
namespace std {
// hash function for ObservationId
template <>
struct hash<KimeraRPGO::ObservationId> {
  std::size_t operator()(const KimeraRPGO::ObservationId& id) const {
    using std::hash;
    return hash<char>()(id.id1) + hash<char>()(id.id2) +
           hash<char>()(id.id1) * hash<char>()(id.id2);
  }
};
}  // namespace std