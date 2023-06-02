#ifndef USERPARAMETER_H
#define USERPARAMETER_H

#include <ocs2_core/Types.h>


namespace ocs2{
namespace wbc{
//Hierarchy and Weight
struct HierNWeig
{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<int, 2, 1> hierarchy;
        Eigen::Matrix<scalar_t, 8, 1>  weight;
};


class UserParameter{
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
        //******************WBC*************************
        vector3_t Kp_body, Kd_body;
        vector3_t Kp_ori, Kd_ori;
        vector3_t Kp_foot, Kd_foot;               //
        vector3_t Kp_foot_lf, Kd_foot_lf;         // Swing Leg Task Space
        vector3_t Kp_LegJoint_lf, Kd_LegJoint_lf; // WBC LegJointMotionControl Task

        vector3_t Kp_foot_lb, Kd_foot_lb;         // Swing Leg Task Space
        vector3_t Kp_LegJoint_lb, Kd_LegJoint_lb; // WBC LegJointMotionControl Task

        vector3_t Kp_foot_rf, Kd_foot_rf;         // Swing Leg Task Space
        vector3_t Kp_LegJoint_rf, Kd_LegJoint_rf; // WBC LegJointMotionControl Task

        vector3_t Kp_foot_rb, Kd_foot_rb;         // Swing Leg Task Space
        vector3_t Kp_LegJoint_rb, Kd_LegJoint_rb; // WBC LegJointMotionControl Task

        vector3_t Kp_joint_lf, Kd_joint_lf;       //For Jonit PD controller
        vector3_t Kp_joint_rf, Kd_joint_rf;       //For Jonit PD controller
        vector3_t Kp_joint_lh, Kd_joint_lh;       //For Jonit PD controller
        vector3_t Kp_joint_rh, Kd_joint_rh;       //For Jonit PD controller
        
        //Physical Parameters
        Eigen::Matrix<scalar_t, 3, 1> TauMax;
        Eigen::Matrix<scalar_t, 1, 3> NContact, HContact, IContact; //the vector of ground
        scalar_t mu;

        HierNWeig hierWeig;//WBC_HO

        //******************PD*************************
        scalar_t Kp_lf_haa_pd, Kd_lf_haa_pd;
        scalar_t Kp_lf_hfe_pd, Kd_lf_hfe_pd;
        scalar_t Kp_lf_kfe_pd, Kd_lf_kfe_pd;

        scalar_t Kp_rf_haa_pd, Kd_rf_haa_pd;
        scalar_t Kp_rf_hfe_pd, Kd_rf_hfe_pd;
        scalar_t Kp_rf_kfe_pd, Kd_rf_kfe_pd;

        scalar_t Kp_lh_haa_pd, Kd_lh_haa_pd;
        scalar_t Kp_lh_hfe_pd, Kd_lh_hfe_pd;
        scalar_t Kp_lh_kfe_pd, Kd_lh_kfe_pd;

        scalar_t Kp_rh_haa_pd, Kd_rh_haa_pd;
        scalar_t Kp_rh_hfe_pd, Kd_rh_hfe_pd;
        scalar_t Kp_rh_kfe_pd, Kd_rh_kfe_pd;

        scalar_t cycle_time;

        //******************SafeGuard*************************
        scalar_t x_delta;
        scalar_t y_delta;
        scalar_t z_delta;
        scalar_t roll_delta;
        scalar_t pitch_delta;
        scalar_t yaw_delta;

        scalar_t Kp_lf_haa_pd_safe, Kd_lf_haa_pd_safe;
        scalar_t Kp_lf_hfe_pd_safe, Kd_lf_hfe_pd_safe;
        scalar_t Kp_lf_kfe_pd_safe, Kd_lf_kfe_pd_safe;

        scalar_t Kp_rf_haa_pd_safe, Kd_rf_haa_pd_safe;
        scalar_t Kp_rf_hfe_pd_safe, Kd_rf_hfe_pd_safe;
        scalar_t Kp_rf_kfe_pd_safe, Kd_rf_kfe_pd_safe;

        scalar_t Kp_lh_haa_pd_safe, Kd_lh_haa_pd_safe;
        scalar_t Kp_lh_hfe_pd_safe, Kd_lh_hfe_pd_safe;
        scalar_t Kp_lh_kfe_pd_safe, Kd_lh_kfe_pd_safe;

        scalar_t Kp_rh_haa_pd_safe, Kd_rh_haa_pd_safe;
        scalar_t Kp_rh_hfe_pd_safe, Kd_rh_hfe_pd_safe;
        scalar_t Kp_rh_kfe_pd_safe, Kd_rh_kfe_pd_safe;
        

        UserParameter(const std::string& filename);
        // ~UserParameter();
};

}
}






#endif