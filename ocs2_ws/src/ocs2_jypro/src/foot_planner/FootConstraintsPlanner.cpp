// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_jypro/foot_planner/FootConstraintsPlanner.h"
#include "ocs2_jypro/gait/MotionPhaseDefinition.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/Lookup.h>

#include <random>

// #include <ocs2_centroidal_model/CentroidalModelInfo.h>

namespace ocs2 {
namespace legged_robot {

FootConstraintsPlanner::FootConstraintsPlanner(PinocchioInterface &pinocchioInterface,
                                               const PinocchioEndEffectorKinematics &endEffectorKinematics,
                                               const CentroidalModelInfo &centroidalModelInfo,
                                               size_t numFeet)
    : pinocchioInterface_(pinocchioInterface),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      centroidalModelInfo_(centroidalModelInfo),
      numFeet_(numFeet) {
    endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
}

vector3_t FootConstraintsPlanner::getFootPlacementNominal(size_t leg, scalar_t time) const {
    const auto index = lookup::findIndexInTimeArray(feetPlacementEvents_[leg], time);
    return feetPlacement_[leg][index];
}

const FootConstraints& FootConstraintsPlanner::getFootPolygonConstraint(size_t leg, scalar_t time) const {
    const auto index = lookup::findIndexInTimeArray(feetPlacementEvents_[leg], time);
    // std::cout << "feetPlacementConstraints_[leg][index]:" << feetPlacementConstraints_[leg][index].A << std::endl;
    if (feetPlacementConstraints_[leg].empty()) {
        printf("feetPlacementConstraints_ is empty");
        throw std::runtime_error("feetPlacementConstraints_ is empty");
        // FootConstraints zeroConstraints;
        // zeroConstraints.A = matrix_t::Zero(1, 3);
        // zeroConstraints.b = vector_t::Zero(1);
        // return zeroConstraints;
    }
    else
        return feetPlacementConstraints_[leg][index];
}

void FootConstraintsPlanner::setMpcTrajectoryAccordingToFootPlacement(const scalar_t initTime, const ModeSchedule& modeSchedule, 
                                                                      TargetTrajectories& targetTrajectories, scalar_t comHeight) {
    const auto &modeSequence = modeSchedule.modeSequence;
    const auto &eventTimes = modeSchedule.eventTimes;

    auto &stateTrajectory = targetTrajectories.stateTrajectory;
    
    const size_t initIndex = lookup::findIndexInTimeArray(eventTimes, initTime);

    // std::cout << targetTrajectories;
    // std::cout << "initTime: " << initTime << "\tinitIndex: " << initIndex << std::endl;

    for(size_t i = 0; i < targetTrajectories.timeTrajectory.size(); i++) {
        const size_t index = lookup::findIndexInTimeArray(eventTimes, targetTrajectories.timeTrajectory[i]);

        matrix_t hootholdsAtEvent = matrix_t::Zero(numFeet_, 3);
        for(size_t leg = 0; leg < numFeet_; leg++) {
            hootholdsAtEvent.row(leg) = feetPlacement_[leg][index];
        }
        vector_t Z = vector_t::Zero(numFeet_);
        Z = hootholdsAtEvent.col(2);

        hootholdsAtEvent.col(2) = vector_t::Ones(hootholdsAtEvent.rows());
        Eigen::JacobiSVD<matrix_t> svd(hootholdsAtEvent, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // not sure if we need to svd.sort()... probably not
        int const nrows(svd.singularValues().rows());
        matrix_t invS;
        invS = matrix_t::Zero(nrows, nrows);
        const float sigmaThreshold = 0.0000001;
        for (int ii(0); ii < nrows; ++ii) {
            if (svd.singularValues().coeff(ii) > sigmaThreshold) {
            invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
            } else {
            // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
            printf("FootPlacement sigular value is too small: %f\n",svd.singularValues().coeff(ii));
            }
        }
        matrix_t H_invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
        
        // ax + by - z + c = 0
        auto estParam =  H_invMatrix * Z; // estParams = [a b c]
        // std::cout << "estParam: " << estParam.transpose() << std::endl;

        stateTrajectory[i][8] = estParam[0]*stateTrajectory[i][6] + estParam[1]*stateTrajectory[i][7] + estParam[2] + comHeight - 0.02; //ax + by + h_ref
        stateTrajectory[i][10] = -std::atan2(estParam[0], 1); //pitch
        stateTrajectory[i][11] = std::atan2(estParam[1], 1); //roll
        // std::cout << "pose des: " << stateTrajectory[i].segment(6,6).transpose() << std::endl;
        // vector3_t desiredEulerAngleYX = -std::atan2(estParam[0], 1), std::atan(estParam[1], 1); //pitch row
        // scalar_t desiredComHeight = desiredEulerAngleZYX[0];
    }

}

vector3_t FootConstraintsPlanner::getCurrentEEPosition(size_t leg, const vector_t& initState) {
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    // NOTE: centroidalModelInfo_ cannot be empty , otherwise the following line will throw an exception.
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(initState, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);
    const auto initFootPosition = endEffectorKinematicsPtr_->getPosition(initState)[leg];
    return initFootPosition;
}

void FootConstraintsPlanner::update(const ModeSchedule &modeSchedule, const TargetTrajectories &targetTrajectories,
                                    scalar_t initTime, const vector_t &initState) {
    const auto &modeSequence = modeSchedule.modeSequence;
    const auto &eventTimes = modeSchedule.eventTimes;

    const size_t initIndex = lookup::findIndexInTimeArray(eventTimes, initTime);

    // std::cout << "initIndex: " << initIndex << std::endl;

    // cut those past sequence
    // std::vector<size_t> modeSequence(modeSequence_.begin() + initIndex, modeSequence_.end());
    // std::vector<scalar_t> eventTimes(eventTimes_.begin() + initIndex, eventTimes_.end());

    // std::vector<size_t> modeSequence(modeSequence_.begin() , modeSequence_.end());
    // std::vector<scalar_t> eventTimes(eventTimes_.begin() , eventTimes_.end());

    // std::cout << "after cut event times:   {" << toDelimitedString(eventTimes) << "}\n";
    // std::cout << "after cut modeSequence:   {" << toDelimitedString(modeSequence) << "}\n";
    // std::cout << modeSchedule;

    const auto eesContactFlagStocks = extractContactFlags(modeSequence);

    feet_array_t<std::vector<int>> startTimesIndices;
    feet_array_t<std::vector<int>> finalTimesIndices;
    for (size_t leg = 0; leg < numFeet_; leg++) {
        std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
    }

    // std::cout << "startTimesIndices: " << toDelimitedString(startTimesIndices[0]) << std::endl;
    // std::cout << "finalTimesIndices: " << toDelimitedString(finalTimesIndices[0]) << std::endl;
    // std::cout << "initState:" << initState.segment(6,6).transpose() << std::endl;
    for (size_t j = 0; j < numFeet_; j++) {
        // using current state to calculate foot placement, where are the liffoff height.
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        // NOTE: centroidalModelInfo_ cannot be empty , otherwise the following line will throw an exception.
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(initState, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);
        const auto initFootPosition = endEffectorKinematicsPtr_->getPosition(initState)[j];

        // using this z position to full of the Sequence.
        //(THIS DOES WORK IN 0.5s PLANNING, BUT NOT SURE IN LONGER PLANNING TIME(1.0s))
        const scalar_array_t liftOffHeightSequence(modeSequence.size(), initFootPosition[2]);
        liftOffHeightSequence_[j] = liftOffHeightSequence;

        // scalar_t middleTime = initIndex == 0 ? eventTimes[initIndex] : (eventTimes[initIndex] + eventTimes[initIndex - 1]) / 2;

        if (eesContactFlagStocks[j][initIndex]) { // currently stance leg // TODO: plan every single middle time of stance
            // std::cout << "stance leg: " << j << "\tinitTime: " << initTime << "\tmiddleTime: " << middleTime << std::endl;
            feetPlacement_[j].clear();
            feetPlacement_[j].reserve(modeSequence.size());

            feetPlacementConstraints_[j].clear();
            feetPlacementConstraints_[j].reserve(modeSequence.size());

            swingHeightSequence_[j].clear();
            swingHeightSequence_[j].reserve(modeSequence.size());

            swingMiddleTimeSequence_[j].clear();
            swingMiddleTimeSequence_[j].reserve(modeSequence.size());


            // save the Z position of the target feet placement
            touchDownHeightSequence_[j].clear();
            touchDownHeightSequence_[j].reserve(modeSequence.size());

            // this line i think should change every prerun.
            // feetPlacementEvents_[j] = eventTimes;
            for (int p = 0; p < modeSequence.size(); ++p) {
                if (!eesContactFlagStocks[j][p]) { // for all swing phases
                    const int swingStartIndex = startTimesIndices[j][p];
                    const int swingFinalIndex = finalTimesIndices[j][p];
                    checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

                    const scalar_t swingStartTime = eventTimes[swingStartIndex];
                    const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

                    // test whether use swingStartTime or swingFinalTime to calculate the desired states.
                    // This maybe affect the max velocity of the base motion, and the horizon of MPC.
                    // Currently, we use swingFinalTime, MPC horizen = 0.5s. max velocity = 0.2m/s.
                    const vector_t desiredstate = targetTrajectories.getDesiredState(swingFinalTime);
                    // const vector_t desiredstate = targetTrajectories.getDesiredState(swingStartTime);

                    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(desiredstate, centroidalModelInfo_));
                    pinocchio::updateFramePlacements(model, data);

                    const auto feetPosition = endEffectorKinematicsPtr_->getPosition(desiredstate)[j];

                    vector3_t footplacement;
                    size_t polygonIndex;
                    // std::cout << "choiceCloestPolygonVertex\n";
                    std::tie(polygonIndex, footplacement) = choiceCloestPolygonVertex(j, feetPosition);
                    feetPlacement_[j].emplace_back(footplacement);
                    touchDownHeightSequence_[j].emplace_back(footplacement[2]);
                    if (p != 0) {
                        liftOffHeightSequence_[j][p - 1] = footplacement[2];
                    }
                    if (p == modeSequence.size() - 1) {
                        liftOffHeightSequence_[j][p] = footplacement[2];
                    }

                    // form constraint
                    ocs2::Polygon polygon;
                    FootConstraints constraint;
                    for (const auto &vertex : legEndEffectorPolygon_[j][polygonIndex]) {
                        polygon.addVertex(vertex.head(2));
                        // std::cout << "vertex: " << vertex.head(2).transpose() << std::endl;
                    }
                    matrix_t A, conA;
                    vector_t b, zeroCol;
                    polygon.convertToInequalityConstraints(A, b);
                    zeroCol.resizeLike(b);
                    zeroCol.setZero();
                    conA.resize(A.rows(), A.cols() + 1);
                    conA << -A, zeroCol;
                    constraint.A = conA;
                    constraint.b = b;
                    // std::cout << "conA:" << constraint.A << "\n";
                    // std::cout << "b:" << constraint.b.transpose() << "\n";
                    feetPlacementConstraints_[j].emplace_back(constraint);
                    swingHeightSequence_[j].emplace_back(swingHeight_[j][polygonIndex]);
                    swingMiddleTimeSequence_[j].emplace_back(swingMiddleTime_[j][polygonIndex]);
                } else { // for a stance leg
                    vector3_t footplacement;
                    size_t polygonIndex;
                    FootConstraints constraint;
                    if(feetPlacement_[j].empty()){
                        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(initState, centroidalModelInfo_));
                        pinocchio::updateFramePlacements(model, data);

                        const auto feetPosition = endEffectorKinematicsPtr_->getPosition(initState)[j];
                        std::tie(polygonIndex, footplacement) = choiceCloestPolygonVertex(j, feetPosition);
                                            // form constraint
                        ocs2::Polygon polygon;
                        for (const auto &vertex : legEndEffectorPolygon_[j][polygonIndex]) {
                            polygon.addVertex(vertex.head(2));
                            // std::cout << "vertex: " << vertex.head(2).transpose() << std::endl;
                        }
                        matrix_t A, conA;
                        vector_t b, zeroCol;
                        polygon.convertToInequalityConstraints(A, b);
                        zeroCol.resizeLike(b);
                        zeroCol.setZero();
                        conA.resize(A.rows(), A.cols() + 1);
                        conA << -A, zeroCol;
                        constraint.A = conA;
                        constraint.b = b;
                        // std::cout << "conA:" << constraint.A << "\n";
                        // std::cout << "b:" << constraint.b.transpose() << "\n";
                    }
                    else{
                        footplacement = feetPlacement_[j].back();
                        constraint = feetPlacementConstraints_[j].back();
                    }
                    // std::cout << "choiceCloestPolygonVertex\n";
                    feetPlacement_[j].emplace_back(footplacement); // TODO: this line should be changed to the nomial.
                    touchDownHeightSequence_[j].emplace_back(footplacement[2]);
                    feetPlacementConstraints_[j].emplace_back(constraint);
                    swingHeightSequence_[j].emplace_back(vector3_t::Zero());
                    swingMiddleTimeSequence_[j].emplace_back(0.1);
                }
            }
        }
        else{
            //copy the previous leg placement according to the current event time.
            std::vector<vector3_t> feetPlacementTemp;
            std::vector<FootConstraints> feetConstraintsTemp;
            std::vector<vector_t> swingHeightSequenceTemp;
            std::vector<scalar_t> swingMiddleTimeSequenceTemp;
            for (int p = 0; p < eventTimes.size(); ++p) {
                size_t index = lookup::findIndexInTimeArray(feetPlacementEvents_[j], eventTimes[p]);
                feetPlacementTemp.emplace_back(feetPlacement_[j][index]);
                feetConstraintsTemp.emplace_back(feetPlacementConstraints_[j][index]);
                swingHeightSequenceTemp.emplace_back(swingHeightSequence_[j][index]);
                swingMiddleTimeSequenceTemp.emplace_back(swingMiddleTimeSequence_[j][index]);
                // touchDownHeightSequence_[j].emplace_back(touchDownHeightSequence_[j][index]);
            }
            feetPlacement_[j] = feetPlacementTemp; 
            feetPlacementConstraints_[j] = feetConstraintsTemp;
            swingHeightSequence_[j] = swingHeightSequenceTemp;
            swingMiddleTimeSequence_[j] = swingMiddleTimeSequenceTemp;
        }
        feetPlacementEvents_[j] = eventTimes;
    }
}

std::pair<int, vector3_t> FootConstraintsPlanner::choiceCloestPolygonVertex(const size_t &footNum,
                                                                            const vector3_t &position) {
    scalar_t minDistance = 100;
    vector3_t minPoint;
    size_t selectedPolygonNum = 0, polygonIndex = 0;
    // std::cout << "in side choiceCloestPolygonVertex" << legEndEffectorPolygon_.size() << "\n" ;
    // for (const auto &polygons : legEndEffectorPolygon_[footNum]) {
    //     // std::cout << "polygons: " << polygons.size() << "\n";
    //     for (const auto &vertex : polygons) {
    //         const scalar_t distance = (vertex - position).norm();
    //         if (distance < minDistance) {
    //             minDistance = distance;
    //             minPoint = vertex;
    //             selectedPolygonNum = polygonIndex;
    //         }
    //     }
    //     ++polygonIndex;
    // }

    for(const auto& point : nominalFoothold_[footNum]){
        const scalar_t distance = (point - position).norm();
        if (distance < minDistance) {
            minDistance = distance;
            minPoint = point;
            selectedPolygonNum = polygonIndex;
        }
        ++polygonIndex;
    }

    return std::make_pair(selectedPolygonNum, minPoint);
}

void FootConstraintsPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t> &phaseIDsStock) {
    const size_t numSubsystems = phaseIDsStock.size();
    if (startIndex < 0) {
        std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
        for (size_t i = 0; i < numSubsystems; i++) {
            std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
        }
        std::cerr << std::endl;

        throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
    }
    if (finalIndex >= numSubsystems - 1) {
        std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
        for (size_t i = 0; i < numSubsystems; i++) {
            std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
        }
        std::cerr << std::endl;

        throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) + " is not defined.");
    }
}

feet_array_t<std::vector<bool>> FootConstraintsPlanner::extractContactFlags(const std::vector<size_t> &phaseIDsStock) const {
    const size_t numPhases = phaseIDsStock.size();

    feet_array_t<std::vector<bool>> contactFlagStock;
    std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

    for (size_t i = 0; i < numPhases; i++) {
        const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
        for (size_t j = 0; j < numFeet_; j++) {
            contactFlagStock[j][i] = contactFlag[j];
        }
    }
    return contactFlagStock;
}

feet_array_t<std::vector<bool>> FootConstraintsPlanner::extractSwingFlags(const std::vector<size_t> &phaseIDsStock) const {
    const size_t numPhases = phaseIDsStock.size();

    feet_array_t<std::vector<bool>> contactFlagStock;
    std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

    for (size_t i = 0; i < numPhases; i++) {
        const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
        for (size_t j = 0; j < numFeet_; j++) {
            contactFlagStock[j][i] = !contactFlag[j];
        }
    }
    return contactFlagStock;
}

std::pair<std::vector<int>, std::vector<int>> FootConstraintsPlanner::updateFootSchedule(const std::vector<bool> &contactFlagStock) {
    const size_t numPhases = contactFlagStock.size();

    std::vector<int> startTimeIndexStock(numPhases, 0);
    std::vector<int> finalTimeIndexStock(numPhases, 0);

    // find the startTime and finalTime indices for swing feet
    for (size_t i = 0; i < numPhases; i++) {
        if (!contactFlagStock[i]) {
            std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
        }
    }
    return {startTimeIndexStock, finalTimeIndexStock};
}

std::pair<int, int> FootConstraintsPlanner::findIndex(size_t index, const std::vector<bool> &contactFlagStock) {
    const size_t numPhases = contactFlagStock.size();

    // skip if it is a stance leg
    if (contactFlagStock[index]) {
        return {0, 0};
    }

    // find the starting time
    int startTimesIndex = -1;
    for (int ip = index - 1; ip >= 0; ip--) {
        if (contactFlagStock[ip]) {
            startTimesIndex = ip;
            break;
        }
    }

    // find the final time
    int finalTimesIndex = numPhases - 1;
    for (size_t ip = index + 1; ip < numPhases; ip++) {
        if (contactFlagStock[ip]) {
            finalTimesIndex = ip - 1;
            break;
        }
    }

    return {startTimesIndex, finalTimesIndex};
}

} // namespace legged_robot
} // namespace ocs2
