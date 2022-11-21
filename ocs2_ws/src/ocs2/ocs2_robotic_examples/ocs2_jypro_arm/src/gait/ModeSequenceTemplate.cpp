#include "ModeSequenceTemplate.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& stream, const ModeSequenceTemplate& modeSequenceTemplate) {
    stream << "Template switching times: {" << toDelimitedString(modeSequenceTemplate.switchingTimes) << "}\n";
    stream << "Template mode sequence:   {" << toDelimitedString(modeSequenceTemplate.modeSequence) << "}\n";
    return stream;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSequenceTemplate loadModeSequenceTemplate(const std::string& filename, const std::string& topicName, bool verbose) {
    std::vector<scalar_t> switchingTimes;
    loadData::loadStdVector(filename, topicName + ".switchingTimes", switchingTimes, verbose);

    std::vector<std::string> modeSequenceString;
    loadData::loadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

    if (switchingTimes.empty() || modeSequenceString.empty()) {
        throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " + topicName + " from " + filename);
    }

    // convert the mode name to mode enum
    std::vector<size_t> modeSequence;
    modeSequence.reserve(modeSequenceString.size());
    for(const auto& modeName : modeSequenceString) {
        modeSequence.push_back(string2ModeNumber(modeName));
    }

    return {switchingTimes, modeSequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::mode_schedule createModeSequenceTemplateMsg(const ModeSequenceTemplate& modeSequenceTemplate) {
    ocs2_msgs::mode_schedule modeScheduleMsg;
    modeScheduleMsg.eventTimes.assign(modeSequenceTemplate.switchingTimes.begin(), modeSequenceTemplate.switchingTimes.end()); //dqwang: modeSequence.size() - 1 ???
    modeScheduleMsg.modeSequence.assign(modeSequenceTemplate.modeSequence.begin(), modeSequenceTemplate.modeSequence.end());
    return modeScheduleMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSequenceTemplate readModeSequenceTemplateMsg(const ocs2_msgs::mode_schedule& modeScheduleMsg) {
    std::vector<scalar_t> switchingTimes(modeScheduleMsg.eventTimes.begin(), modeScheduleMsg.eventTimes.end());
    std::vector<size_t> modeSequence(modeScheduleMsg.modeSequence.begin(), modeScheduleMsg.modeSequence.end());
    return {switchingTimes, modeSequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Gait toGait(const ModeSequenceTemplate& modeSequenceTemplate) {
    const auto startTime = modeSequenceTemplate.switchingTimes.front();
    const auto endTime = modeSequenceTemplate.switchingTimes.back();
    Gait gait;
    gait.duration = endTime - startTime;
    // Events: from time -> phase
    gait.eventPhases.reserve(modeSequenceTemplate.switchingTimes.size());
    std::for_each(modeSequenceTemplate.switchingTimes.begin() + 1, modeSequenceTemplate.switchingTimes.end() - 1,
                  [&](scalar_t eventTime) { gait.eventPhases.push_back((eventTime - startTime) / gait.duration); });
    // Modes:
    gait.modeSequence = modeSequenceTemplate.modeSequence;
    assert(isValidGait(gait));
    return gait;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule loadModeSchedule(const std::string& filename, const std::string& topicName, bool verbose) {
    std::vector<scalar_t> eventTimes;
    loadData::loadStdVector(filename, topicName + ".eventTimes", eventTimes, verbose);

    std::vector<std::string> modeSequenceString;
    loadData::loadStdVector(filename, topicName + ".modeSequence", modeSequenceString, verbose);

    if(modeSequenceString.empty()) {
        throw std::runtime_error("[loadModeSchedule] fail to load : " + topicName + " from " + filename);
    }

    // convert the mode name to mode enum
    std::vector<size_t> modeSequence;
    modeSequence.reserve(modeSequenceString.size());
    for (const auto& modeName : modeSequenceString) {
        modeSequence.push_back(string2ModeNumber(modeName));
    }

    return {eventTimes, modeSequence};
}

} // namespace legged_robot
} // namespace ocs2