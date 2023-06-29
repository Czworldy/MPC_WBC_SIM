#include "ocs2_jypro/foot_planner/MinimumJerkSeventhOrderSpline.h"

namespace ocs2 {
namespace legged_robot {

MinimumJerkSeventhOrderSplineSolver::MinimumJerkSeventhOrderSplineSolver(scalar_t miniumOrder)
    : miniumOrder_(miniumOrder),
      qp_(16, 13, qpOASES::HST_POSDEF) {
    H_.setZero();
    const int coefficientNum = 8;
    for (int k = 0; k < 2; ++k) {
        for (int i = miniumOrder_; i < coefficientNum; ++i) {
            for (int j = miniumOrder_; j < coefficientNum; ++j)
                H_(k * coefficientNum + i, k * coefficientNum + j) = 1.0 * factorial(i) / factorial(i - miniumOrder_) * factorial(j) / factorial(j - miniumOrder_) / (i + j - 2 * miniumOrder_ + 1);
        }
    }
    H_.diagonal().array() += scalar_t(1e-6); // make it POSDEF
    qpOASES::Options options;
    options.setToMPC(); // Fast
    options.printLevel = qpOASES::PL_HIGH;
    qp_.setOptions(options);

    Eigen::Matrix<scalar_t, 6, 8> Aeq_block_topLeft;
    Aeq_block_topLeft.setZero();
    Aeq_block_topLeft(0, 0) = 1;
    Aeq_block_topLeft(1, 1) = 1;
    Aeq_block_topLeft(2, 2) = 2;
    Aeq_block_topLeft.row(3) << 1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125;
    Aeq_block_topLeft.row(4) << 1, 1, 1, 1, 1, 1, 1, 1;
    Aeq_block_topLeft.row(5) << 0, 1, 2, 3, 4, 5, 6, 7;

    Eigen::Matrix<scalar_t, 6, 8> Aeq_block_bottomRight = Aeq_block_topLeft;
    Aeq_block_bottomRight.row(2) << 0, 0, 2, 6, 12, 20, 30, 42;

    Aeq_.setZero();

    Aeq_.topLeftCorner(6, 8) = Aeq_block_topLeft;
    Aeq_.block(6, 8, 6, 8) = Aeq_block_bottomRight;
    Aeq_.row(12) << 0, 0, 2, 6, 12, 20, 30, 42, 0, 0, -2, 0, 0, 0, 0, 0;

    std::cout << "Aeq_:\n"
              << Aeq_ << "\n";
    std::cout << "H_:\n"
              << H_ << "\n";

    int nWsr = 10;
    vector_t lbA = (vector_t(13) << 0, 0.2, 1, 0.12, 0.15, 0, 0.15, 0, 0, 0.1, 0, 0, 0).finished();
    vector_t g = vector_t::Zero(16);
    qp_.init(H_.data(), g.data(), Aeq_.data(), nullptr, nullptr, lbA.data(), lbA.data(), nWsr);

    vector_t primalSolution(H_.rows());
    qp_.getPrimalSolution(primalSolution.data());
    // std::cout << "primalSolution: " << primalSolution.transpose() << "\n";
}

vector_t MinimumJerkSeventhOrderSplineSolver::solveCoffectient(Node liftOff, scalar_t leftMidHeight, Node apex,
                                                               scalar_t rightMidHeight, Node touchDown) {
    // p1 v1 a1 p2 p3 v3 p3 v3 a5 p4 p5 v5 0

    vector_t lbA = (vector_t(13) << liftOff.position, liftOff.velocity, liftOff.acceleration,
                    leftMidHeight, apex.position, apex.velocity,
                    apex.position, apex.velocity, touchDown.acceleration,
                    rightMidHeight, touchDown.position, touchDown.velocity,
                    0)
                       .finished();
    // std::cout << "lbA: " << lbA.transpose() << "\n";
    int nWsr = 10;
    vector_t g = vector_t::Zero(16);
    qp_.hotstart(g.data(), nullptr, nullptr, lbA.data(), lbA.data(), nWsr);

    vector_t primalSolution(H_.rows());
    qp_.getPrimalSolution(primalSolution.data());
    // std::cout << "primalSolution: " << primalSolution.transpose() << "\n";

    return primalSolution;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SenvenOrderSpline::position(scalar_t time) const {
    scalar_t tn = normalizedTime(time);
    vector_t vec_tn = (vector_t(8) << 1.0, tn, tn * tn, std::pow(tn, 3), std::pow(tn, 4), std::pow(tn, 5), std::pow(tn, 6), std::pow(tn, 7)).finished();
    return (vec_tn.transpose() * c_)(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SenvenOrderSpline::velocity(scalar_t time) const {
    scalar_t tn = normalizedTime(time);
    vector_t vec_tn = (vector_t(8) << 0.0, 1.0, 2.0 * tn, 3.0 * pow(tn, 2), 4.0 * pow(tn, 3), 5.0 * pow(tn, 4), 6.0 * pow(tn, 5), 7.0 * pow(tn, 6)).finished();
    return (vec_tn.transpose() * c_)(0) / dt_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SenvenOrderSpline::acceleration(scalar_t time) const {
    scalar_t tn = normalizedTime(time);
    vector_t vec_tn = (vector_t(8) << 0.0, 0.0, 2.0, 6.0 * tn, 12.0 * pow(tn, 2), 20.0 * pow(tn, 3), 30.0 * pow(tn, 4), 42.0 * pow(tn, 5)).finished();
    return (vec_tn.transpose() * c_)(0) / (dt_ * dt_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SenvenOrderSpline::normalizedTime(scalar_t t) const {
    assert(t >= t0_);
    assert(t <= t1_);
    return (t - t0_) / dt_;
}

} // namespace legged_robot
} // namespace ocs2