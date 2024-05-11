#include "ocs2_wbc/UserParameter.h"
#include "ocs2_core/misc/LoadData.h"

namespace ocs2{
namespace wbc{

UserParameter::UserParameter(const std::string& filename){
    // const std::string filename = "/home/yjy/jy_control_test/include/PARAMETER/UserParameter_sdk_ws.info";
    const std::string pdCfg = "PD";
    const std::string wbcCfg = "WBC";
    const std::string safeGuardCfg = "SafeGuard";
    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    loadData::loadPtreeValue(pt, cycle_time, "cycle_time", verbose);
    loadData::loadPtreeValue(pt, mu, wbcCfg + ".mu", verbose);
    std::vector<scalar_t> tauMax, tauMin; 
    loadData::loadStdVector(filename, wbcCfg + ".TauMax", tauMax, verbose);
    for(int i(0); i < 3; i++) {
        TauMax[i] = tauMax[i];
    }
    std::vector<scalar_t> kd_body, kp_body, kd_ori, kp_ori, kd_foot, kp_foot;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_body", kd_body, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_body", kp_body, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_ori", kd_ori, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_ori", kp_ori, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot", kd_foot, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot", kp_foot, verbose);
    for(int i(0); i < 3; i++) {
        Kd_body[i] = kd_body[i];
        Kp_body[i] = kp_body[i];
        Kd_ori[i] = kd_ori[i];
        Kp_ori[i] = kp_ori[i];
        Kd_foot[i] = kd_foot[i];
        Kp_foot[i] = kp_foot[i];
    }
    std::vector<scalar_t> kd_foot_lf, kp_foot_lf;
    std::vector<scalar_t> kd_LegJoint_lf, kp_LegJoint_lf;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot_lf", kd_foot_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot_lf", kp_foot_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_LegJoint_lf", kd_LegJoint_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_LegJoint_lf", kp_LegJoint_lf, verbose);
    for(int i(0); i < 3; i++) {
        Kd_foot_lf[i] = kd_foot_lf[i];
        Kp_foot_lf[i] = kp_foot_lf[i];

        Kd_LegJoint_lf[i] = kd_LegJoint_lf[i];
        Kp_LegJoint_lf[i] = kp_LegJoint_lf[i];
    }

    std::vector<scalar_t> kd_foot_rf, kp_foot_rf;
    std::vector<scalar_t> kd_LegJoint_rf, kp_LegJoint_rf;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot_rf", kd_foot_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot_rf", kp_foot_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_LegJoint_rf", kd_LegJoint_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_LegJoint_rf", kp_LegJoint_rf, verbose);
    for(int i(0); i < 3; i++) {
        Kd_foot_rf[i] = kd_foot_rf[i];
        Kp_foot_rf[i] = kp_foot_rf[i];

        Kd_LegJoint_rf[i] = kd_LegJoint_rf[i];
        Kp_LegJoint_rf[i] = kp_LegJoint_rf[i];
    }

    std::vector<scalar_t> kd_foot_lb, kp_foot_lb;
    std::vector<scalar_t> kd_LegJoint_lb, kp_LegJoint_lb;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot_lb", kd_foot_lb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot_lb", kp_foot_lb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_LegJoint_lb", kd_LegJoint_lb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_LegJoint_lb", kp_LegJoint_lb, verbose);
    for(int i(0); i < 3; i++) {
        Kd_foot_lb[i] = kd_foot_lb[i];
        Kp_foot_lb[i] = kp_foot_lb[i];

        Kd_LegJoint_lb[i] = kd_LegJoint_lb[i];
        Kp_LegJoint_lb[i] = kp_LegJoint_lb[i];
    }

    std::vector<scalar_t> kd_foot_rb, kp_foot_rb;
    std::vector<scalar_t> kd_LegJoint_rb, kp_LegJoint_rb;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot_rb", kd_foot_rb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot_rb", kp_foot_rb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_LegJoint_rb", kd_LegJoint_rb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_LegJoint_rb", kp_LegJoint_rb, verbose);
    for(int i(0); i < 3; i++) {
        Kd_foot_rb[i] = kd_foot_rb[i];
        Kp_foot_rb[i] = kp_foot_rb[i];

        Kd_LegJoint_rb[i] = kd_LegJoint_rb[i];
        Kp_LegJoint_rb[i] = kp_LegJoint_rb[i];
    }

    std::vector<scalar_t> kd_joint_lf, kp_joint_lf, kd_joint_rf, kp_joint_rf, kd_joint_lh, kp_joint_lh, kd_joint_rh, kp_joint_rh;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_joint_lf", kd_joint_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_joint_lf", kp_joint_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_joint_rf", kd_joint_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_joint_rf", kp_joint_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_joint_lh", kd_joint_lh, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_joint_lh", kp_joint_lh, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_joint_rh", kd_joint_rh, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_joint_rh", kp_joint_rh, verbose);
    for (int i(0); i < 3; i++) {
        Kd_joint_lf[i] = kd_joint_lf[i];
        Kp_joint_lf[i] = kp_joint_lf[i];
        Kd_joint_rf[i] = kd_joint_rf[i];
        Kp_joint_rf[i] = kp_joint_rf[i];
        Kd_joint_lh[i] = kd_joint_lh[i];
        Kp_joint_lh[i] = kp_joint_lh[i];
        Kd_joint_rh[i] = kd_joint_rh[i];
        Kp_joint_rh[i] = kp_joint_rh[i];
    }
    std::vector<scalar_t> nContact, hContact, iContact;
    loadData::loadStdVector(filename, wbcCfg + ".NContact", nContact, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".HContact", hContact, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".IContact", iContact, verbose);
    for(int i(0); i < 3; i++) {
        NContact[i] = nContact[i];
        HContact[i] = hContact[i];
        IContact[i] = iContact[i];
    }
    std::vector<int> hierarchy;
    std::vector<scalar_t> weight;
    loadData::loadStdVector(filename, wbcCfg + ".hierarchy", hierarchy, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".weight", weight, verbose);
    for(int i(0); i < hierWeig.hierarchy.rows(); i++) {
        hierWeig.hierarchy[i] = hierarchy[i];
    }
    for(int i(0); i < hierWeig.weight.rows(); i++) {
        hierWeig.weight[i] = weight[i];
    }

    // std::cerr << "\n[dqwang: UserParameter<scalar_t>::UserParameter] hierWeig.hierarchy: " << hierWeig.hierarchy << "\n";
    std::cerr << "\n[dqwang: UserParameter<scalar_t>::UserParameter] hierWeig.weight: " << hierWeig.weight.transpose() << "\n";

    loadData::loadPtreeValue(pt, Kd_lf_haa_pd, pdCfg + ".Kd_lf_haa_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_lf_haa_pd, pdCfg + ".Kp_lf_haa_pd", verbose);
    loadData::loadPtreeValue(pt, Kd_lf_hfe_pd, pdCfg + ".Kd_lf_hfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_lf_hfe_pd, pdCfg + ".Kp_lf_hfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kd_lf_kfe_pd, pdCfg + ".Kd_lf_kfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_lf_kfe_pd, pdCfg + ".Kp_lf_kfe_pd", verbose);

    loadData::loadPtreeValue(pt, Kd_rf_haa_pd, pdCfg + ".Kd_rf_haa_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_rf_haa_pd, pdCfg + ".Kp_rf_haa_pd", verbose);
    loadData::loadPtreeValue(pt, Kd_rf_hfe_pd, pdCfg + ".Kd_rf_hfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_rf_hfe_pd, pdCfg + ".Kp_rf_hfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kd_rf_kfe_pd, pdCfg + ".Kd_rf_kfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_rf_kfe_pd, pdCfg + ".Kp_rf_kfe_pd", verbose);

    loadData::loadPtreeValue(pt, Kd_lh_haa_pd, pdCfg + ".Kd_lh_haa_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_lh_haa_pd, pdCfg + ".Kp_lh_haa_pd", verbose);
    loadData::loadPtreeValue(pt, Kd_lh_hfe_pd, pdCfg + ".Kd_lh_hfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_lh_hfe_pd, pdCfg + ".Kp_lh_hfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kd_lh_kfe_pd, pdCfg + ".Kd_lh_kfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_lh_kfe_pd, pdCfg + ".Kp_lh_kfe_pd", verbose);

    loadData::loadPtreeValue(pt, Kd_rh_haa_pd, pdCfg + ".Kd_rh_haa_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_rh_haa_pd, pdCfg + ".Kp_rh_haa_pd", verbose);
    loadData::loadPtreeValue(pt, Kd_rh_hfe_pd, pdCfg + ".Kd_rh_hfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_rh_hfe_pd, pdCfg + ".Kp_rh_hfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kd_rh_kfe_pd, pdCfg + ".Kd_rh_kfe_pd", verbose);
    loadData::loadPtreeValue(pt, Kp_rh_kfe_pd, pdCfg + ".Kp_rh_kfe_pd", verbose);


    //***************************SafeGuard************************
    loadData::loadPtreeValue(pt, x_delta,     safeGuardCfg + ".x_delta", verbose);
    loadData::loadPtreeValue(pt, y_delta,     safeGuardCfg + ".y_delta", verbose);
    loadData::loadPtreeValue(pt, z_delta,     safeGuardCfg + ".z_delta", verbose);
    loadData::loadPtreeValue(pt, roll_delta,  safeGuardCfg + ".roll_delta", verbose);
    loadData::loadPtreeValue(pt, pitch_delta, safeGuardCfg + ".pitch_delta", verbose);
    loadData::loadPtreeValue(pt, yaw_delta,   safeGuardCfg + ".yaw_delta", verbose);

    loadData::loadPtreeValue(pt, Kd_lf_haa_pd_safe, safeGuardCfg + ".Kd_lf_haa_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_lf_haa_pd_safe, safeGuardCfg + ".Kp_lf_haa_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kd_lf_hfe_pd_safe, safeGuardCfg + ".Kd_lf_hfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_lf_hfe_pd_safe, safeGuardCfg + ".Kp_lf_hfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kd_lf_kfe_pd_safe, safeGuardCfg + ".Kd_lf_kfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_lf_kfe_pd_safe, safeGuardCfg + ".Kp_lf_kfe_pd_safe", verbose);

    loadData::loadPtreeValue(pt, Kd_rf_haa_pd_safe, safeGuardCfg + ".Kd_rf_haa_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_rf_haa_pd_safe, safeGuardCfg + ".Kp_rf_haa_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kd_rf_hfe_pd_safe, safeGuardCfg + ".Kd_rf_hfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_rf_hfe_pd_safe, safeGuardCfg + ".Kp_rf_hfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kd_rf_kfe_pd_safe, safeGuardCfg + ".Kd_rf_kfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_rf_kfe_pd_safe, safeGuardCfg + ".Kp_rf_kfe_pd_safe", verbose);

    loadData::loadPtreeValue(pt, Kd_lh_haa_pd_safe, safeGuardCfg + ".Kd_lh_haa_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_lh_haa_pd_safe, safeGuardCfg + ".Kp_lh_haa_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kd_lh_hfe_pd_safe, safeGuardCfg + ".Kd_lh_hfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_lh_hfe_pd_safe, safeGuardCfg + ".Kp_lh_hfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kd_lh_kfe_pd_safe, safeGuardCfg + ".Kd_lh_kfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_lh_kfe_pd_safe, safeGuardCfg + ".Kp_lh_kfe_pd_safe", verbose);

    loadData::loadPtreeValue(pt, Kd_rh_haa_pd_safe, safeGuardCfg + ".Kd_rh_haa_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_rh_haa_pd_safe, safeGuardCfg + ".Kp_rh_haa_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kd_rh_hfe_pd_safe, safeGuardCfg + ".Kd_rh_hfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_rh_hfe_pd_safe, safeGuardCfg + ".Kp_rh_hfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kd_rh_kfe_pd_safe, safeGuardCfg + ".Kd_rh_kfe_pd_safe", verbose);
    loadData::loadPtreeValue(pt, Kp_rh_kfe_pd_safe, safeGuardCfg + ".Kp_rh_kfe_pd_safe", verbose);

    loadData::loadPtreeValue(pt, contactPhaseThreshold, wbcCfg + ".contactPhaseThreshold", verbose);
    std::cout << "###########contactPhaseThreshold###########: " << contactPhaseThreshold << "\n";

    std::cerr << "\nUserParamerter Construction Finished!\n";
}


}
}




    