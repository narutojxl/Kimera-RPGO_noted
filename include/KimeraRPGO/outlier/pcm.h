/*
Pairwise Consistency Maximization (PCM)
Backend solver class (Robust Pose Graph Optimizer)
author: Yun Chang, Luca Carlone
 */

#pragma once

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <math.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "KimeraRPGO/logger.h"
#include "KimeraRPGO/outlier/OutlierRemoval.h"
#include "KimeraRPGO/utils/geometry_utils.h"
#include "KimeraRPGO/utils/graph_utils.h"

namespace KimeraRPGO {

/* ------------------------------------------------------------------------ */
// Defines the behaviour of this backend.
enum class FactorType {
  UNCLASSIFIED = 0,
  ODOMETRY = 1,  //
  FIRST_LANDMARK_OBSERVATION = 2,
  LOOP_CLOSURE = 3,
  // both between poses and landmark re-observations (may be more than 1)
  NONBETWEEN_FACTORS = 4,  // not handled by PCM (may be more than 1)
};

/**
*@param poseT: can be gtsam::Pose2 or Pose3 for 3D vs 3D
*@param T:  can be PoseWithCovariance or PoseWithDistance based on
*           If using Pcm or PcmDistance
*/
template <class poseT, template <class> class T>
class Pcm : public OutlierRemoval {
 public:
  Pcm(double threshold1,
      double threshold2,
      const std::vector<char>& special_symbols = std::vector<char>())
      : OutlierRemoval(),
        threshold1_(threshold1),
        threshold2_(threshold2),
        special_symbols_(special_symbols),
        total_lc_(0),
        total_good_lc_(0) {
    // check if templated value valid
    BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<poseT>));
  }
  ~Pcm() = default;
  // initialize with odometry detect threshold and pairwise consistency
  // threshold
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  double threshold1_;
  double threshold2_;

  // NonlinearFactorGraph storing all odometry factors
  gtsam::NonlinearFactorGraph nfg_odom_;

  // NonlinearFactorGraph storing all NonBetweenFactors
  gtsam::NonlinearFactorGraph nfg_special_;

  // storing loop closures and its adjacency matrix
  std::unordered_map<ObservationId, Measurements> loop_closures_;

  // trajectory maping robot prefix to its keys and poses
  std::unordered_map<char, Trajectory<poseT, T>> odom_trajectories_;
  //char: 每个机器人有个char id标志

  // these are the symbols corresponding to landmarks
  std::vector<char> special_symbols_;

  // storing landmark measurements and its adjacency matrix
  std::unordered_map<gtsam::Key, Measurements> landmarks_;

  size_t total_lc_, total_good_lc_;

 public:
  size_t getNumLC() { return total_lc_; }
  size_t getNumLCInliers() { return total_good_lc_; }

  /*! \brief Process new measurements and reject outliers
   *  process the new measurements and update the "good set" of measurements
   *  - new_factors: factors from the new measurements
   *  - new_values: linearization point of the new measurements
   *  - nfg: the factors after processing new measurements and outlier removal
   *  - values: the values after processing new measurements and outlier removal
   *  - returns: boolean of if optimization should be called or not
   */
  bool removeOutliers(const gtsam::NonlinearFactorGraph& new_factors,
                      const gtsam::Values& new_values,
                      gtsam::NonlinearFactorGraph* output_nfg,
                      gtsam::Values* output_values) override {
    // store new values:
    output_values->insert(
        new_values);  // - store latest pose in values_ (note:
                      // values_ is the optimized estimate,
                      // while trajectory is the odom estimate)

    // Check values to initialize a trajectory in odom_trajectories if needed
    if (new_factors.size() == 0) {
      // Done and nothing to optimize
      return false;
    }

    bool do_optimize = false;
    // ==============================================================================
    gtsam::NonlinearFactorGraph loop_closure_factors;
    for (size_t i = 0; i < new_factors.size(); i++) {
      if (NULL == new_factors[i]) continue;

      // we first classify the current factors into the following categories:
      FactorType type = FactorType::UNCLASSIFIED;

      //* current logic: The factors handled by outlier rejection are the between
      //* factors. Between factors can be classified broadly into odometry, loop
      //* closures, and landmark observations

      // check if factor is a between factor
      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT>>(//之所以用boost接口是因为new_factors[i]类型是boost::shared_ptr<NonlinearFactor>
              new_factors[i])) {//下行转换后，判断是否为nullptr
        // specifically what outlier rejection handles
        gtsam::Key from_key = new_factors[i]->front();
        gtsam::Key to_key = new_factors[i]->back();
        gtsam::Symbol from_symb(from_key);
        gtsam::Symbol to_symb(to_key);
        if (isSpecialSymbol(from_symb.chr()) ||
            isSpecialSymbol(to_symb.chr())) {  // is it a landmark observation
          // if includes values, it is the first observation
          if (new_values.exists(from_key) || new_values.exists(to_key)) {
            type = FactorType::FIRST_LANDMARK_OBSERVATION;
          } else {
            // re-observation: counts as loop closure
            type = FactorType::LOOP_CLOSURE;
          }
        } else if (from_key + 1 == to_key && new_values.exists(to_key)) { //里程计factor: 相邻两个node之间
          // Note that here we assume we receive odometry incrementally
          type = FactorType::ODOMETRY;  // just regular odometry
        } else {
          type = FactorType::LOOP_CLOSURE; //包括同一个机器人的nodes之间；不同机器人的nodes之间
        }
      } else {
        type = FactorType::NONBETWEEN_FACTORS;  // unrecognized factor, not
                                                // checked for outlier rejection
      }

      // ==============================================================================
      // handle differently depending on type
      switch (type) {
        case FactorType::ODOMETRY:  // odometry, do not optimize
        {
          updateOdom(new_factors[i], *output_values); //把该odom factor加入到odom_trajectories_变量中
        } break;

        case FactorType::FIRST_LANDMARK_OBSERVATION:  // landmark measurement,
                                                      // initialize
        {
          if (debug_) log<INFO>("New landmark observed");
          Measurements newMeasurement;
          newMeasurement.factors.add(new_factors[i]);
          newMeasurement.consistent_factors.add(new_factors[i]);
          gtsam::Symbol symb(new_values.keys()[0]);
          landmarks_[symb] = newMeasurement;
          total_lc_++;
        } break;

        case FactorType::LOOP_CLOSURE: {//包括同一个机器人的nodes之间；不同机器人的nodes之间；还包括re-observation landmark
          if (new_factors[i]->front() != new_factors[i]->back()) {
            // add the the loop closure factors and process them together
            loop_closure_factors.add(new_factors[i]);
          } else {
            log<WARNING>("Attempting to close loop against self");
          }
        } break;

        case FactorType::NONBETWEEN_FACTORS: {
          nfg_special_.add(new_factors[i]);
          do_optimize = true;
        } break;

        default:  // the remainders are specical loop closure cases, includes
                  // the "UNCLASSIFIED" case
        {
          nfg_special_.add(new_factors[i]);
          do_optimize = true;
        }
      }  // end switch
    }//end all factors


    if (loop_closure_factors.size() > 0) {
      // update inliers
      parseAndIncrementAdjMatrix(loop_closure_factors, *output_values);
      // output values is just used to sanity check the keys

      findInliers(); //根据所有闭环的邻接矩阵和对应的马氏距离，计算哪些闭环是inlier闭环
                     //同理计算inlier的landmark re-observation(位姿node与地图中的landmark不是第一次观测时)
      //! Find inliers with Pairwise consistent measurement set maximization
      // J. Mangelson, D. Dominic, R. Eustice, and R. Vasudevan. 
      // Pairwise Consistent Measurement Set Maximization for Robust Multi-robot Map Merging. 
      // IEEE Intl. Conf. on Robotics and Automation (ICRA), 2018.

      do_optimize = true;
    }

    *output_nfg = buildGraphToOptimize(); //添加inlier的闭环和inlier landmark re-observation到因子图中
    return do_optimize;
  }  // end reject outliers



  /*! \brief save the PCM data
   *  saves the distance matrix (final) and also the clique size info
   *  - folder_path: path to directory to save results in
   */
  void saveData(std::string folder_path) override {
    // TODO(Yun) save max clique results
    // saveDistanceMatrix(folder_path);
    // saveCliqueSizeData(folder_path);
  }

  /*! \brief remove the last loop closure based on observation ID
   * and update the factors.
   * For example if Observation id is Obsid('a','c'), method
   * removes the last loop closure between robots a and c
   */
  void removeLastLoopClosure(ObservationId id,
                             gtsam::NonlinearFactorGraph* updated_factors) {
    if (loop_closures_.find(id) == loop_closures_.end()) {
      return;  // No loop closures in this container
    }
    // Update the measurements (delete last measurement)
    size_t numLC = loop_closures_[id].adj_matrix.rows();
    if (numLC <= 0) {
      return;  // No more loop closures
    }

    loop_closures_[id].factors.erase(
        std::prev(loop_closures_[id].factors.end()));
    if (loop_closures_[id].factors.size() < 2) {
      loop_closures_[id].consistent_factors = loop_closures_[id].factors;
    } else {
      // Update adjacent and distance matrix
      loop_closures_[id].adj_matrix =
          loop_closures_[id].adj_matrix.block(0, 0, numLC - 1, numLC - 1);
      loop_closures_[id].dist_matrix =
          loop_closures_[id].dist_matrix.block(0, 0, numLC - 1, numLC - 1);

      // Update the inliers
      std::vector<int> inliers_idx;
      size_t num_inliers =
          findMaxCliqueHeu(loop_closures_[id].adj_matrix, &inliers_idx);

      loop_closures_[id].consistent_factors =
          gtsam::NonlinearFactorGraph();  // reset
      // update inliers, or consistent factors, according to max clique result
      for (size_t i = 0; i < num_inliers; i++) {
        loop_closures_[id].consistent_factors.add(
            loop_closures_[id].factors[inliers_idx[i]]);
      }
    }

    *updated_factors = buildGraphToOptimize();
    return;
  }

 protected:
  /*! \brief goes through the loop closures and updates the corresponding
   * adjacency matrices(邻接矩阵), in preparation for max clique
   */
  void parseAndIncrementAdjMatrix(
      const gtsam::NonlinearFactorGraph& new_factors, //包括同一个机器人的nodes之间；不同机器人的nodes之间；还包括re-observation landmark
      const gtsam::Values& output_values) {
    for (size_t i = 0; i < new_factors.size(); i++) {
      // iterate through the factors
      // double check again that these are between factors
      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT>>(
              new_factors[i])) {
        // regular loop closure.
        // in this case we should run consistency check to see if loop closure
        // is good
        // * odometric consistency check (will only compare against odometry
        // - if loop fails this, we can just drop it)

        // extract between factor
        gtsam::BetweenFactor<poseT> nfg_factor = //该loop factor
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT>>(
                new_factors[i]);

        if (!output_values.exists(nfg_factor.keys().front()) ||
            !output_values.exists(nfg_factor.keys().back())) {
          log<WARNING>("Cannot add loop closure with non-existing keys");
          continue;
        }

        // check if it is a landmark measurement loop closure
        gtsam::Symbol symbfrnt(nfg_factor.front());
        gtsam::Symbol symbback(nfg_factor.back());
        if (isSpecialSymbol(symbfrnt.chr()) ||
            isSpecialSymbol(symbback.chr())) { //机器人和landmark之间的re-observation landmark factor
          // it is landmark loop closure
          gtsam::Key landmark_key =
              (isSpecialSymbol(symbfrnt.chr()) ? nfg_factor.front()
                                               : nfg_factor.back());

          if (debug_)
            log<INFO>("loop closing with landmark %1%") %
                gtsam::DefaultKeyFormatter(landmark_key);

          landmarks_[landmark_key].factors.add(nfg_factor);
          total_lc_++;

          // grow adj matrix
          incrementLandmarkAdjMatrix(landmark_key);

        } else {
          // It is a proper loop closures
          double odom_dist;
          bool odom_consistent = false;
          if (symbfrnt.chr() == symbback.chr()) {//同一个机器人的nodes之间
            odom_consistent = isOdomConsistent(nfg_factor, &odom_dist);
                             //闭环检测得到的两个nodes之间的约束“减去”odom factor计算得到的约束，该delta_pose的马氏距离是否小于设置的阈值
          } else {//不同机器人的nodes之间
            // odom consistency check only for intrarobot loop closures
            odom_consistent = true;
          }

          if (odom_consistent) {
            if (debug_)
              log<INFO>("loop closure between keys %1% and %2%") %
                  gtsam::DefaultKeyFormatter(nfg_factor.front()) %
                  gtsam::DefaultKeyFormatter(nfg_factor.back());

            ObservationId obs_id(symbfrnt.chr(), symbback.chr()); 
            //ObservationId: 机器人a与机器人a，机器人a与机器人b，机器人a与机器人c...

            // detect which inter or intra robot loop closure this belongs to
            loop_closures_[obs_id].factors.add(nfg_factor); //添加2种类型
                                                            //(1)保存一个机器人nodes之间odom consistent loop factor 
                                                            //(2)保存不同机器人nodes之间loop factor
            total_lc_++;

            incrementAdjMatrix(obs_id, nfg_factor); //nfg_factor刚添加到loop_closures_[obs_id].factors::factors_[]的最后一个
            //检查每条闭环与剩余所有的闭环是否consistent，产生consistent邻接矩阵和对应的马氏距离矩阵

          } else {
            if (debug_)
              log<WARNING>(
                  "Discarded loop closure (inconsistent with odometry)");
            continue;  // discontinue since loop closure not consistent with
                       // odometry
          }
        }//end loop factor

      } else {
        // add as special loop closure
        // the remainders are speical loop closure cases
        nfg_special_.add(new_factors[i]);
      }
    }//end all loop factor
  }

  // check if a character is a special symbol as defined in constructor
  // (typically these are the landmarks)
  bool isSpecialSymbol(const char& symb) const {
    for (size_t i = 0; i < special_symbols_.size(); i++) {
      if (special_symbols_[i] == symb) return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  // update the odometry: add new measurements to odometry trajectory tree
  // 把odom factor加入到odom_trajectories_变量中
  void updateOdom(const gtsam::NonlinearFactor::shared_ptr& new_factor,
                  const gtsam::Values& output_values) {
    // here we have values for reference checking and initialization if needed
    gtsam::BetweenFactor<poseT> odom_factor =
        *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT>>(new_factor);

    nfg_odom_.add(odom_factor);  // - store factor in nfg_odom_

    // update trajectory(compose last value with new odom value)
    gtsam::Key new_key = odom_factor.keys().back(); //连接odom factor的后一个node的id

    // extract prefix
    gtsam::Symbol sym = gtsam::Symbol(new_key);
    char prefix = sym.chr(); //轨迹的id号，每个机器人轨迹有一个id号

    // construct pose with covariance for odometry measurement
    T<poseT> odom_delta(odom_factor);

    // Now get the latest pose in trajectory and compose
    gtsam::Key prev_key = odom_factor.keys().front(); //连接odom factor的前一个node的id
    T<poseT> prev_pose;
    try {
      prev_pose = odom_trajectories_[prefix].poses[prev_key]; //前面node的值和covariance
    } catch (...) {
      if (odom_trajectories_.find(prefix) == odom_trajectories_.end()) {
        // prefix has not been seen before, add
        T<poseT> initial_pose;
        initial_pose.pose = output_values.at<poseT>(prev_key);
        // populate trajectories
        odom_trajectories_[prefix].poses[prev_key] = initial_pose;
      } else {
        log<WARNING>("Attempted to add odom to non-existing key. ");
      }
    }

    // compose latest pose to odometry for new pose
    T<poseT> new_pose = prev_pose.compose(odom_delta);

    // add to trajectory
    odom_trajectories_[prefix].poses[new_key] = new_pose; //后面node的值和covariance
  }

  /* *******************************************************************************
   */
  /*
   * odometry consistency check specialized to the PoseWithCovariance class
   */
  bool checkOdomConsistent(const PoseWithCovariance<poseT>& result,
                           double* dist) const {
    *dist = result.mahalanobis_norm();  // for PCM
    if (debug_) log<INFO>("odometry consistency distance: %1%") % *dist;
    if (*dist < threshold1_) {
      return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  /*
   * odometry consistency check specialized to the PoseWithNode class
   */
  bool checkOdomConsistent(const PoseWithNode<poseT>& result, double* dist) {
    // For PcmSimple
    *dist = result.avg_trans_norm();
    double rot_dist = result.avg_rot_norm();

    if (debug_)
      log<INFO>("odometry consistency translation distance: %1%") % *dist;
    if (debug_)
      log<INFO>("odometry consistency rotation distance: %1%") % rot_dist;
    if (*dist < threshold1_ && rot_dist < threshold2_) {
      return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  /*
   * general interface for odometry consistency check (both PCM and distance
   * version)
   */
  bool isOdomConsistent(const gtsam::BetweenFactor<poseT>& lc_factor, //同一个机器人nodes之间的loop factor
                        double* dist) {
    // say: loop is between pose i and j
    gtsam::Key key_i = lc_factor.keys().front();  // extract the keys
    gtsam::Key key_j = lc_factor.keys().back();
    gtsam::Symbol symb_i = gtsam::Symbol(key_i);
    gtsam::Symbol symb_j = gtsam::Symbol(key_j);

    T<poseT> pij_odom, pji_lc, result;

    if (symb_i.chr() != symb_j.chr()) {
      log<WARNING>(
          "Only check for odmetry consistency for intrarobot loop closures");
    }
    pij_odom = odom_trajectories_[symb_i.chr()].getBetween(key_i, key_j); //由odom factors构造的odom_trajectories_，得到的两个nodes之间的变换

    // get pij_lc = (Tij_lc, Covij_lc) from factor
    pji_lc = T<poseT>(lc_factor).inverse(); //闭环检测模块计算的两个nodes之间的变换

    // check consistency (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc)
    result = pij_odom.compose(pji_lc);
    // if (debug_) result.pose.print("odom consistency check: ");

    return checkOdomConsistent(result, dist); //计算result的马氏距离是否小于设置的阈值: 如果小于返回true,否则false。
  }

  /* *******************************************************************************
   */
  /*
   * pairwise loop consistency check specialized to the PoseWithCovariance class
   * TODO: delete this function, rename checkOdomConsistent to
   * checkPairwiseConsistency and let it take a third argument (the threshold)
   */
  bool checkLoopConsistent(const PoseWithCovariance<poseT>& result,
                           double* dist) {
    *dist = result.mahalanobis_norm();
    if (*dist < threshold2_) {
      return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  /*
   * pairwise loop consistency check specialized to the PoseWithNode class
   * TODO: delete this function, rename checkOdomConsistent to
   * checkPairwiseConsistency and let it take a third argument (the threshold)
   */
  bool checkLoopConsistent(const PoseWithNode<poseT>& result, double* dist) {
    *dist = result.avg_trans_norm();
    double rot_dist = result.avg_rot_norm();
    if (*dist < threshold1_ && rot_dist < threshold2_) {
      return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  /*
   * general interface for loop consistency check (both PCM and distance
   * version) inputs are 2 loop closures (a,b) and (c,d), where a,b,c,d are keys
   */
  bool areLoopsConsistent(const gtsam::BetweenFactor<poseT>& a_lcBetween_b, //闭环检测模块得到的a, b之间的变换
                          const gtsam::BetweenFactor<poseT>& c_lcBetween_d, //闭环检测模块得到的c, d之间的变换
                          double* dist) {
    // check if two loop closures are consistent
    // say: loop closure 1 is (a,b)
    gtsam::Key key_a = a_lcBetween_b.keys().front();
    gtsam::Key key_b = a_lcBetween_b.keys().back();
    // say: loop closure 2 is (c,d)
    gtsam::Key key_c = c_lcBetween_d.keys().front();
    gtsam::Key key_d = c_lcBetween_d.keys().back();

    T<poseT> a_lc_b, c_lc_d;
    a_lc_b = T<poseT>(a_lcBetween_b);
    c_lc_d = T<poseT>(c_lcBetween_d);
    gtsam::Symbol symb_a = gtsam::Symbol(key_a);
    gtsam::Symbol symb_b = gtsam::Symbol(key_b);
    gtsam::Symbol symb_c = gtsam::Symbol(key_c);
    gtsam::Symbol symb_d = gtsam::Symbol(key_d);

    // make sure a and c has same prefix (from same robot)
    if (symb_a.chr() != symb_c.chr()) {
      // switch c and d if needed
      gtsam::Key temp = key_c;
      key_c = key_d;
      key_d = temp;
      symb_c = gtsam::Symbol(key_c);
      symb_d = gtsam::Symbol(key_d);
    }
    // find odometry from a to c
    if (symb_a.chr() != symb_c.chr()) {
      log<WARNING>("Attempting to get odometry between different trajectories");
    }
    T<poseT> a_odom_c =
        odom_trajectories_[symb_a.chr()].getBetween(key_a, key_c); //假设a, c属于机器人a; 两个node之间的变换由odom_trajectories_查询得到
    // find odometry from d to b
    if (symb_b.chr() != symb_d.chr()) {
      log<WARNING>("Attempting to get odometry between different trajectories");
    }
    T<poseT> b_odom_d =
        odom_trajectories_[symb_b.chr()].getBetween(key_b, key_d); //假设b, d属于机器人b；同上。

    // check that d to b pose is consistent with pose from b to d
    T<poseT> a_path_d, d_path_b, loop;
    a_path_d = a_odom_c.compose(c_lc_d);
    d_path_b = a_path_d.inverse().compose(a_lc_b);
    loop = d_path_b.compose(b_odom_d);
    return checkLoopConsistent(loop, dist); //检测闭环的consistent
  }

  /* *******************************************************************************
   */
  /*
   * augment adjacency matrix with an extra (pose-pose) loop closure
   */
  void incrementAdjMatrix(const ObservationId& id, //机器人a与机器人a，机器人a与机器人b，机器人a与机器人c...
                          const gtsam::BetweenFactor<poseT>& factor) { //2种类型
                                                                       //(1)保存一个机器人nodes之间odom consistent loop factor 
                                                                       //(2)保存不同机器人nodes之间loop factor                                                           
    // pairwise consistency check (will also compare other loops - if loop
    // fails we still store it, but not include in the optimization)
    // -- add 1 row and 1 column to lc_adjacency_matrix_;
    // -- populate extra row and column by testing pairwise consistency of new
    // lc against all previous ones
    // -- compute max clique (done in the findInliers function)
    // -- add loops in max clique to a local variable nfg_good_lc (done in the
    // updateOutputGraph function) Using correspondence rowId (size_t, in
    // adjacency matrix) to slot id (size_t, id of that lc in nfg_lc)
    if (loop_closures_.find(id) == loop_closures_.end()) {
      // does not exist yet, add
      Measurements new_measurements;
      loop_closures_[id] = new_measurements;
    }
    size_t num_lc = //关于该ObservationId本次调用时有多少条闭环约束
        loop_closures_[id].factors.size();  // number of loop closures so far,
                                            // including the one we just added
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
          loop_closures_[id].adj_matrix;
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
          loop_closures_[id].dist_matrix;

      // now iterate through the previous loop closures and fill in last row +
      // col of adjacency
      // 当前的闭环factor与剩余所有的闭环factors依次检测consistent
      for (size_t i = 0; i < num_lc - 1; i++) {  // compare it against all others; 
        gtsam::BetweenFactor<poseT> factor_i =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT>>(
                loop_closures_[id].factors[i]); //loop_closures_[id].factors[num_lc - 1]是当前factor
        // check consistency
        double mah_distance;
        bool consistent = areLoopsConsistent(factor_i, factor, &mah_distance); 
        new_dst_matrix(num_lc - 1, i) = mah_distance;
        new_dst_matrix(i, num_lc - 1) = mah_distance; //与每条检测的闭环计算马氏距离
        if (consistent) {
          new_adj_matrix(num_lc - 1, i) = 1; //与哪条闭环consistent，邻接矩阵的相应位置为1
          new_adj_matrix(i, num_lc - 1) = 1;
        }
      }
    }
    loop_closures_[id].adj_matrix = new_adj_matrix; //检查每条闭环与剩余所有的闭环consistent，产生consistent邻接矩阵和对应的马氏距离
    loop_closures_[id].dist_matrix = new_dst_matrix;
  }



  /* *******************************************************************************
   */
  /*
   * augment adjacency matrix for a landmark loop closure
   */
  void incrementLandmarkAdjMatrix(const gtsam::Key& ldmk_key) {
    // pairwise consistency check for landmarks
    size_t num_lc = landmarks_[ldmk_key].factors.size();  // number measurements
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
          landmarks_[ldmk_key].adj_matrix;
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
          landmarks_[ldmk_key].dist_matrix;

      // now iterate through the previous loop closures and fill in last row +
      // col of adjacency
      gtsam::BetweenFactor<poseT>
          factor_jl =  // latest landmark loop closure: to be checked
          *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT>>(
              landmarks_[ldmk_key].factors[num_lc - 1]);

      // check it against all others
      for (size_t i = 0; i < num_lc - 1; i++) {
        gtsam::BetweenFactor<poseT> factor_il =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT>>(
                landmarks_[ldmk_key].factors[i]);

        // check consistency
        gtsam::Key keyi = factor_il.keys().front();
        gtsam::Key keyj = factor_jl.keys().front();

        if (keyi == ldmk_key || keyj == ldmk_key) {
          log<WARNING>(
              "Landmark observations should be connected pose -> "
              "landmark, discarding");
          return;
        }

        // factors are (i,l) and (j,l) and connect poses i,j to a landmark l
        T<poseT> i_pose_l, j_pose_l;
        i_pose_l = T<poseT>(factor_il);
        j_pose_l = T<poseT>(factor_jl);

        gtsam::Symbol symb_i = gtsam::Symbol(keyi);
        gtsam::Symbol symb_j = gtsam::Symbol(keyj);

        // find odometry from 1a to 2a
        if (symb_i.chr() != symb_j.chr()) {
          log<WARNING>(
              "Attempting to get odometry between different trajectories");
        }
        T<poseT> i_odom_j =
            odom_trajectories_[symb_i.chr()].getBetween(keyi, keyj);

        // check that lc_1 pose is consistent with pose from 1a to 1b
        T<poseT> i_path_l, loop;
        i_path_l = i_odom_j.compose(j_pose_l);
        loop = i_path_l.inverse().compose(i_pose_l);

        double dist;
        bool consistent = checkLoopConsistent(loop, &dist);

        new_dst_matrix(num_lc - 1, i) = dist;
        new_dst_matrix(i, num_lc - 1) = dist;
        if (consistent) {
          new_adj_matrix(num_lc - 1, i) = 1;
          new_adj_matrix(i, num_lc - 1) = 1;
        }
      }
    }
    landmarks_[ldmk_key].adj_matrix = new_adj_matrix;
    landmarks_[ldmk_key].dist_matrix = new_dst_matrix;
  }



  /* *******************************************************************************
   */
  /*
   * Based on adjacency(邻接矩阵) matrices, call maxclique(最大团) to extract inliers
   */
  void findInliers() {
    if (debug_) log<INFO>("total loop closures registered: %1%") % total_lc_;
    total_good_lc_ = 0;

    // iterate through loop closures and find inliers
    std::unordered_map<ObservationId, Measurements>::iterator it = loop_closures_.begin();
    while (it != loop_closures_.end()) {
      std::vector<int> inliers_idx;
      it->second.consistent_factors = gtsam::NonlinearFactorGraph();  // reset

      // find max clique
      size_t num_inliers = findMaxCliqueHeu(it->second.adj_matrix, &inliers_idx);//TODO: 算法核心找最大团

      // update inliers, or consistent factors, according to max clique result
      for (size_t i = 0; i < num_inliers; i++) {
        it->second.consistent_factors.add(it->second.factors[inliers_idx[i]]); //找出inlier的闭环factor
      }
      it++;
      total_good_lc_ = total_good_lc_ + num_inliers;
    }


    // iterate through landmarks and find inliers
    std::unordered_map<gtsam::Key, Measurements>::iterator it_ldmrk = landmarks_.begin();
    while (it_ldmrk != landmarks_.end()) {
      std::vector<int> inliers_idx;
      it_ldmrk->second.consistent_factors = gtsam::NonlinearFactorGraph();  // reset

      // find max clique
      size_t num_inliers = findMaxCliqueHeu(it_ldmrk->second.adj_matrix, &inliers_idx);//找出inlier的landmark re-observation

      // update inliers, or consistent factors, according to max clique result
      for (size_t i = 0; i < num_inliers; i++) {
        it_ldmrk->second.consistent_factors.add(it_ldmrk->second.factors[inliers_idx[i]]);
      }
      it_ldmrk++;
      total_good_lc_ = total_good_lc_ + num_inliers;
    }
    if (debug_) log<INFO>("number of inliers: %1%") % total_good_lc_;
  }



  /* *******************************************************************************
   */
  /*
   * update the set of inliers to be outputted
   */
  gtsam::NonlinearFactorGraph buildGraphToOptimize() {
    gtsam::NonlinearFactorGraph output_nfg;  // reset
    output_nfg.add(nfg_odom_);               // add the odometry factors

    // add the good loop closures
    std::unordered_map<ObservationId, Measurements>::iterator it = loop_closures_.begin();
    while (it != loop_closures_.end()) {
      output_nfg.add(it->second.consistent_factors);
      it++;
    }
    // add the good loop closures associated with landmarks
    std::unordered_map<gtsam::Key, Measurements>::iterator it_ldmrk = landmarks_.begin();
    while (it_ldmrk != landmarks_.end()) {
      output_nfg.add(it_ldmrk->second.consistent_factors);
      it_ldmrk++;
    }

    output_nfg.add(nfg_special_);  // still need to update the class overall factorgraph
    return output_nfg;
  }
};

typedef Pcm<gtsam::Pose2, PoseWithCovariance> Pcm2D;
typedef Pcm<gtsam::Pose3, PoseWithCovariance> Pcm3D;
typedef Pcm<gtsam::Pose2, PoseWithNode> PcmSimple2D;
typedef Pcm<gtsam::Pose3, PoseWithNode> PcmSimple3D;

}  // namespace KimeraRPGO