#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ManipulatorModelInfo.h"

namespace ocs2 {
namespace legged_robot {

EndEffectorTrackMode loadEndEffectorTrackMode(const std::string& configFilePath, const std::string& fieldName) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<EndEffectorTrackMode>(type);
}

} // namespace legged_robot
} // namespace ocs2