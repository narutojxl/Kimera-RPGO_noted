/*
Robust solver class
author: Yun Chang, Luca Carlone
*/

#include "KimeraRPGO/RobustSolver.h"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/dataset.h>

#include "KimeraRPGO/logger.h"
#include "KimeraRPGO/outlier/pcm.h"
#include "KimeraRPGO/utils/type_utils.h"

namespace KimeraRPGO {

typedef std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> GraphAndValues;

RobustSolver::RobustSolver(const RobustSolverParams& params)
    : GenericSolver(params.solver, params.specialSymbols) {
  switch (params.outlierRemovalMethod) {
    case OutlierRemovalMethod::NONE: {
      outlier_removal_ =
          nullptr;  // only returns optimize true or optimize false
    } break;
    case OutlierRemovalMethod::PCM2D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<Pcm2D>(params.pcm_odomThreshold,
                                         params.pcm_lcThreshold,
                                         params.specialSymbols);
    } break;
    case OutlierRemovalMethod::PCM3D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<Pcm3D>(params.pcm_odomThreshold, //同一个机器人的nodes之间的闭环consistent check：检测得到的两个nodes之间的约束“减去”odom factor计算得到的约束，该delta_pose的马氏距离的阈值
                                         params.pcm_lcThreshold,   //机器人a的node之间的闭环consistent检测: 一个闭环要与剩余的所有闭环做检测； 检测方法和上面的一样，计算马氏距离
                                         params.specialSymbols);   //机器人a与机器人b的node之间的闭环consistent检测：一个闭环要与剩余的所有闭环做检测；
    } break;
    case OutlierRemovalMethod::PCM_Simple2D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<PcmSimple2D>(params.pcmDist_transThreshold,
                                               params.pcmDist_rotThreshold,
                                               params.specialSymbols);
    } break;
    case OutlierRemovalMethod::PCM_Simple3D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<PcmSimple3D>(params.pcmDist_transThreshold,
                                               params.pcmDist_rotThreshold,
                                               params.specialSymbols);
    } break;
    default: {
      log<WARNING>("Undefined outlier removal method");
      exit(EXIT_FAILURE);
    }
  }

  // toggle verbosity
  switch (params.verbosity) {
    case Verbosity::UPDATE: {
      if (outlier_removal_) outlier_removal_->setQuiet();
    } break;
    case Verbosity::QUIET: {
      if (outlier_removal_)
        outlier_removal_->setQuiet();  // set outlier removal quiet
      setQuiet();                      // set solver quiet
    } break;
    case Verbosity::VERBOSE: {
      log<INFO>("Starting RobustSolver.");
    } break;
    default: {
      log<WARNING>("Unrecognized verbosity. Automatically setting to UPDATE. ");
    }
  }
}

void RobustSolver::optimize() {
  if (solver_type_ == Solver::LM) {
    gtsam::LevenbergMarquardtParams params;
    if (debug_) {
      params.setVerbosityLM("SUMMARY");
      log<INFO>("Running LM");
    }
    params.diagonalDamping = true;
    values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
  } else if (solver_type_ == Solver::GN) {
    gtsam::GaussNewtonParams params;
    if (debug_) {
      params.setVerbosity("ERROR");
      log<INFO>("Running GN");
    }
    values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
  } else {
    log<WARNING>("Unsupported Solver");
    exit(EXIT_FAILURE);
  }
}

void RobustSolver::forceUpdate(const gtsam::NonlinearFactorGraph& nfg,
                               const gtsam::Values& values) {
  if (outlier_removal_) {
    outlier_removal_->removeOutliers(nfg, values, &nfg_, &values_);
  } else {
    addAndCheckIfOptimize(nfg, values);
  }
  // optimize
  optimize();
}

void RobustSolver::update(const gtsam::NonlinearFactorGraph& factors,
                          const gtsam::Values& values) {
  bool do_optimize;
  if (outlier_removal_) {
    do_optimize = outlier_removal_->removeOutliers(factors, values, &nfg_, &values_); 
    // nfg_：   移除outlier测量后的gtsam::NonlinearFactorGraph
    // values_: 和原来的values保持一致
  } else {
    do_optimize = addAndCheckIfOptimize(factors, values);
  }

  if (do_optimize) optimize();  // optimize once after loading
  return;
}

void RobustSolver::removeLastLoopClosure(char prefix_1, char prefix_2) {
  ObservationId id(prefix_1, prefix_2);
  if (outlier_removal_) {
    // removing loop closure so values should not change
    outlier_removal_->removeLastLoopClosure(id, &nfg_); //*移除指定的闭环
  } else {
    removeLastFactor();
  }

  optimize();
  return;
}

void RobustSolver::saveData(std::string folder_path) const {
  std::string g2o_file_path = folder_path + "/result.g2o";
  gtsam::writeG2o(nfg_, values_, g2o_file_path);
  if (outlier_removal_) {
    outlier_removal_->saveData(folder_path);
  }
}

}  // namespace KimeraRPGO
