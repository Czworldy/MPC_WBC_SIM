#include "ModelSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace legged_robot {

ModelSettings loadModelSettings(const std::string& filename, const std::string& fieldname, bool verbose) {
    ModelSettings modelSettings;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename,pt);

    if (verbose) {
        std::cerr << "\n #### Legged Robot Model Settings: ";
        std::cerr << "\n #### ===========================================================================\n";
    }

    loadData::loadPtreeValue(pt, modelSettings.positionErrorGain, fieldname + ".positionErrorGain", verbose);
    loadData::loadPtreeValue(pt, modelSettings.phaseTransitionStanceTime, fieldname + ".phaseTransitionStanceTime", verbose);

    loadData::loadPtreeValue(pt, modelSettings.verboseCppAd, fieldname + ".verboseCppAd", verbose);
    loadData::loadPtreeValue(pt, modelSettings.recompileLibrariesCppAd, fieldname + ".recompileLibrariesCppAd", verbose);
    loadData::loadPtreeValue(pt, modelSettings.modelFolderCppAd, fieldname + ".modelFolderCppAd", verbose);

    if (verbose) {
        std::cerr << " #### =============================================================================" << std::endl;
    }

    return modelSettings;
}

} // namespace legged_robot
} // namespace ocs2