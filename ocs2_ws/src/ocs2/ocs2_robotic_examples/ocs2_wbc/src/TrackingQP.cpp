// created by czworldy 2023/04/17


#include "ocs2_wbc/TrackingQP.h"

namespace ocs2 {
namespace wbc {

TrackingQP::TrackingQP(const Task& trackingCost, const Task& constraints) {
  // std::cout << "trackingCost: \n " << std::endl;
  // trackingCost.print();
  // std::cout << "constraints: \n " << std::endl;
  // constraints.print();

  h_ = trackingCost.a_.transpose()*trackingCost.a_; // no need times 0.5
  c_ = -trackingCost.a_.transpose()*trackingCost.b_;

  //qpInequalityMatrix
  // d_ = Task::concatenateMatrices(equalityConstraints_.a_, inequalityConstraints_.d_);
  // qpInequalityUpperBound_ = Task::concatenateVectors(equalityConstraints_.b_, inequalityConstraints_.f_);
  // vector_t infinityLowerBound(inequalityConstraints_.f_.rows());
  // infinityLowerBound.setConstant(-1e30);
  // qpInequalityLowerBound_ = Task::concatenateVectors(equalityConstraints_.b_, infinityLowerBound);
  d_ = Task::concatenateMatrices(constraints.a_, constraints.d_);
  qpInequalityUpperBound_ = Task::concatenateVectors(constraints.b_, constraints.f_);
  vector_t infinityLowerBound(constraints.f_.rows());
  infinityLowerBound.setConstant(-1e30);
  qpInequalityLowerBound_ = Task::concatenateVectors(constraints.b_, infinityLowerBound);

  solveProblem();
}

void TrackingQP::setQpProblem(const Task& trackingCost, const Task& constraints, bool isInitRun) {
  h_ = trackingCost.a_.transpose()*trackingCost.a_; // no need times 0.5
  c_ = -trackingCost.a_.transpose()*trackingCost.b_;

  d_ = Task::concatenateMatrices(constraints.a_, constraints.d_);
  qpInequalityUpperBound_ = Task::concatenateVectors(constraints.b_, constraints.f_);
  vector_t infinityLowerBound(constraints.f_.rows());
  infinityLowerBound.setConstant(-qpOASES::INFTY);
  qpInequalityLowerBound_ = Task::concatenateVectors(constraints.b_, infinityLowerBound);

  solveSqpProblem(isInitRun);
}

TrackingQP::TrackingQP(size_t nVar, size_t nC) {
  sqpProblemPtr_ = std::make_shared<qpOASES::SQProblem>(nVar, nC, qpOASES::HST_SEMIDEF);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  sqpProblemPtr_->setOptions(options);
}


//example.init(H, g, A, lb, ub, lbA, ubA, nWSR);
void TrackingQP::solveProblem() {
  auto qpProblem = qpOASES::QProblem(h_.cols(), qpInequalityUpperBound_.size(), qpOASES::HST_SEMIDEF); // maybe hessian is positive semi-definite.
  // std::cout <<"qp size:" << h_.cols() << "\t" << qpInequalityUpperBound_.size() << "\n";
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  qpProblem.setOptions(options);
  int nWsr = 100; // 20

  qpBenchmark_.startTimer();
  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, qpInequalityLowerBound_.data(),
                  qpInequalityUpperBound_.data(), nWsr);
  vector_t qpSol(h_.rows());

  qpProblem.getPrimalSolution(qpSol.data());
  qpBenchmark_.endTimer();

  primalSolution_ = qpSol;
}

void TrackingQP::solveSqpProblem(bool isInitRun) {
  qpBenchmark_.startTimer();
  int nWsr = 100; // 20

  if(isInitRun)
    sqpProblemPtr_->init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, qpInequalityLowerBound_.data(),
                  qpInequalityUpperBound_.data(), nWsr);
  else
    sqpProblemPtr_->hotstart(h_.data(), c_.data(), d_.data(), nullptr, nullptr, qpInequalityLowerBound_.data(),
                  qpInequalityUpperBound_.data(), nWsr);
  vector_t qpSol(h_.rows());

  sqpProblemPtr_->getPrimalSolution(qpSol.data());
  qpBenchmark_.endTimer();

  primalSolution_ = qpSol;
}

}
}