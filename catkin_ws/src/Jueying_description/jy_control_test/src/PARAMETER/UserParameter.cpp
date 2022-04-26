#include "UserParameter.h"
#include "LoadData.h"

template<typename T>
UserParameter<T>::UserParameter(){
    const std::string filename = "/home/yjy/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/include/PARAMETER/UserParameter.info";
    const std::string pdCfg = "PD";
    const std::string wbcCfg = "WBC";
    const std::string safeGuardCfg = "SafeGuard";
    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    loadData::loadPtreeValue(pt, cycle_time, "cycle_time", verbose);
    loadData::loadPtreeValue(pt, mu, wbcCfg + ".mu", verbose);
    std::vector<T> tauMax, tauMin; 
    loadData::loadStdVector(filename, wbcCfg + ".TauMax", tauMax, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".TauMin", tauMin, verbose);
    for(int i(0); i < 12; i++) {
        TauMax[i] = tauMax[i];
        TauMin[i] = tauMin[i];
    }
    std::vector<T> kd_body, kp_body, kd_ori, kp_ori, kd_foot, kp_foot;
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
    std::vector<T> kd_body_lf, kp_body_lf, kd_ori_lf, kp_ori_lf, kd_foot_lf, kp_foot_lf;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_body_lf", kd_body_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_body_lf", kp_body_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_ori_lf",  kd_ori_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_ori_lf",  kp_ori_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot_lf", kd_foot_lf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot_lf", kp_foot_lf, verbose);
    for(int i(0); i < 3; i++) {
        Kd_body_lf[i] = kd_body_lf[i];
        Kp_body_lf[i] = kp_body_lf[i];
        Kd_ori_lf[i] = kd_ori_lf[i];
        Kp_ori_lf[i] = kp_ori_lf[i];
        Kd_foot_lf[i] = kd_foot_lf[i];
        Kp_foot_lf[i] = kp_foot_lf[i];
    }
    std::vector<T> kd_body_rf, kp_body_rf, kd_ori_rf, kp_ori_rf, kd_foot_rf, kp_foot_rf;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_body_rf", kd_body_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_body_rf", kp_body_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_ori_rf",  kd_ori_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_ori_rf",  kp_ori_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot_rf", kd_foot_rf, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot_rf", kp_foot_rf, verbose);
    for(int i(0); i < 3; i++) {
        Kd_body_rf[i] = kd_body_rf[i];
        Kp_body_rf[i] = kp_body_rf[i];
        Kd_ori_rf[i] = kd_ori_rf[i];
        Kp_ori_rf[i] = kp_ori_rf[i];
        Kd_foot_rf[i] = kd_foot_rf[i];
        Kp_foot_rf[i] = kp_foot_rf[i];
    }
    std::vector<T> kd_body_lb, kp_body_lb, kd_ori_lb, kp_ori_lb, kd_foot_lb, kp_foot_lb;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_body_lb", kd_body_lb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_body_lb", kp_body_lb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_ori_lb",  kd_ori_lb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_ori_lb",  kp_ori_lb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot_lb", kd_foot_lb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot_lb", kp_foot_lb, verbose);
    for(int i(0); i < 3; i++) {
        Kd_body_lb[i] = kd_body_lb[i];
        Kp_body_lb[i] = kp_body_lb[i];
        Kd_ori_lb[i] = kd_ori_lb[i];
        Kp_ori_lb[i] = kp_ori_lb[i];
        Kd_foot_lb[i] = kd_foot_lb[i];
        Kp_foot_lb[i] = kp_foot_lb[i];
    }
    std::vector<T> kd_body_rb, kp_body_rb, kd_ori_rb, kp_ori_rb, kd_foot_rb, kp_foot_rb;
    loadData::loadStdVector(filename, wbcCfg + ".Kd_body_rb", kd_body_rb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_body_rb", kp_body_rb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_ori_rb",  kd_ori_rb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_ori_rb",  kp_ori_rb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kd_foot_rb", kd_foot_rb, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".Kp_foot_rb", kp_foot_rb, verbose);
    for(int i(0); i < 3; i++) {
        Kd_body_rb[i] = kd_body_rb[i];
        Kp_body_rb[i] = kp_body_rb[i];
        Kd_ori_rb[i] = kd_ori_rb[i];
        Kp_ori_rb[i] = kp_ori_rb[i];
        Kd_foot_rb[i] = kd_foot_rb[i];
        Kp_foot_rb[i] = kp_foot_rb[i];
    }
    std::vector<T> kd_joint_lf, kp_joint_lf, kd_joint_rf, kp_joint_rf, kd_joint_lh, kp_joint_lh, kd_joint_rh, kp_joint_rh;
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

    std::vector<T> nContact, hContact, iContact;
    loadData::loadStdVector(filename, wbcCfg + ".NContact", nContact, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".HContact", hContact, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".IContact", iContact, verbose);
    for(int i(0); i < 3; i++) {
        NContact[i] = nContact[i];
        HContact[i] = hContact[i];
        IContact[i] = iContact[i];
    }
    std::vector<int> hierarchy;
    std::vector<T> weight;
    loadData::loadStdVector(filename, wbcCfg + ".hierarchy", hierarchy, verbose);
    loadData::loadStdVector(filename, wbcCfg + ".weight", weight, verbose);
    for(int i(0); i < hierWeig.hierarchy.rows(); i++) {
        hierWeig.hierarchy[i] = hierarchy[i];
    }
    for(int i(0); i < hierWeig.weight.rows(); i++) {
        hierWeig.weight[i] = weight[i];
    }

    std::cerr << "\n[dqwang: UserParameter<T>::UserParameter] hierWeig.hierarchy: " << hierWeig.hierarchy << "\n";
    std::cerr << "\n[dqwang: UserParameter<T>::UserParameter] hierWeig.weight: " << hierWeig.weight << "\n";

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

    std::cerr << "\nUserParamerter Construction Finished!\n";
}

template<typename T>
UserParameter<T>::~UserParameter(){}


template class UserParameter<double>;
template class UserParameter<float>;




    