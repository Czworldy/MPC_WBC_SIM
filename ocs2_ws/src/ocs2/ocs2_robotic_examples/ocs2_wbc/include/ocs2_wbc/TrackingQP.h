#pragma once

#include "ocs2_wbc/Task.h"
#include <qpOASES.hpp>
#include <ocs2_core/misc/Benchmark.h>
#include <memory>

namespace ocs2{
namespace wbc {

class TrackingQP
{
public:
  TrackingQP(size_t nVar, size_t nC);
  TrackingQP(const Task& trackingCost, const Task& Constraints);
  ~TrackingQP(){
    // std::cerr << "\n### TrackingQP Benchmarking";
    // std::cerr << "\n###   Maximum : " << qpBenchmark_.getMaxIntervalInMilliseconds() << "[ms].";
    // std::cerr << "\n###   Average : " << qpBenchmark_.getAverageInMilliseconds() << "[ms].";
  }
  void setQpProblem(const Task& trackingCost, const Task& Constraints, bool isInitRun);
  vector_t getSolutions() { return primalSolution_; }
  
private:
  void solveProblem();
  void solveSqpProblem(bool isInitRun);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  vector_t c_, qpInequalityUpperBound_, qpInequalityLowerBound_;
  vector_t primalSolution_;
  std::shared_ptr<qpOASES::SQProblem> sqpProblemPtr_;

  benchmark::RepeatedTimer qpBenchmark_;
};




}
}