#ifndef USERPARAMETER_H
#define USERPARAMETER_H

#include <cppTypes.h>

//Hierarchy and Weight
template<typename T>
struct HierNWeig
{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<int, 2, 1> hierarchy;
        Eigen::Matrix<T, 8, 1>  weight;
};


template<typename T>
class UserParameter{
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        //******************WBC*************************
        Vec31<T> Kp_body, Kd_body;
        Vec31<T> Kp_ori, Kd_ori;
        Vec31<T> Kp_foot, Kd_foot;

        Vec31<T> Kp_body_lf, Kd_body_lf;
        Vec31<T> Kp_ori_lf, Kd_ori_lf;
        Vec31<T> Kp_foot_lf, Kd_foot_lf;
        Vec31<T> Kp_LegJoint_lf, Kd_LegJoint_lf; // WBC LegJointMotionControl Task

        Vec31<T> Kp_body_lb, Kd_body_lb;
        Vec31<T> Kp_ori_lb, Kd_ori_lb;
        Vec31<T> Kp_foot_lb, Kd_foot_lb;
        Vec31<T> Kp_LegJoint_lb, Kd_LegJoint_lb; // WBC LegJointMotionControl Task

        Vec31<T> Kp_body_rf, Kd_body_rf;
        Vec31<T> Kp_ori_rf, Kd_ori_rf;
        Vec31<T> Kp_foot_rf, Kd_foot_rf;
        Vec31<T> Kp_LegJoint_rf, Kd_LegJoint_rf; // WBC LegJointMotionControl Task

        Vec31<T> Kp_body_rb, Kd_body_rb;
        Vec31<T> Kp_ori_rb, Kd_ori_rb;
        Vec31<T> Kp_foot_rb, Kd_foot_rb;
        Vec31<T> Kp_LegJoint_rb, Kd_LegJoint_rb; // WBC LegJointMotionControl Task

        Vec31<T> Kp_joint_lf, Kd_joint_lf;
        Vec31<T> Kp_joint_rf, Kd_joint_rf;
        Vec31<T> Kp_joint_lh, Kd_joint_lh;
        Vec31<T> Kp_joint_rh, Kd_joint_rh;
        
        //Physical Parameters
        Vec12<T> TauMax, TauMin;
        Mat13<T> NContact, HContact, IContact; //the vector of ground
        T mu;

        HierNWeig<T> hierWeig;//WBC_HO

        //******************PD*************************
        T Kp_lf_haa_pd, Kd_lf_haa_pd;
        T Kp_lf_hfe_pd, Kd_lf_hfe_pd;
        T Kp_lf_kfe_pd, Kd_lf_kfe_pd;

        T Kp_rf_haa_pd, Kd_rf_haa_pd;
        T Kp_rf_hfe_pd, Kd_rf_hfe_pd;
        T Kp_rf_kfe_pd, Kd_rf_kfe_pd;

        T Kp_lh_haa_pd, Kd_lh_haa_pd;
        T Kp_lh_hfe_pd, Kd_lh_hfe_pd;
        T Kp_lh_kfe_pd, Kd_lh_kfe_pd;

        T Kp_rh_haa_pd, Kd_rh_haa_pd;
        T Kp_rh_hfe_pd, Kd_rh_hfe_pd;
        T Kp_rh_kfe_pd, Kd_rh_kfe_pd;

        T cycle_time;

        //******************SafeGuard*************************
        T x_delta;
        T y_delta;
        T z_delta;
        T roll_delta;
        T pitch_delta;
        T yaw_delta;

        T Kp_lf_haa_pd_safe, Kd_lf_haa_pd_safe;
        T Kp_lf_hfe_pd_safe, Kd_lf_hfe_pd_safe;
        T Kp_lf_kfe_pd_safe, Kd_lf_kfe_pd_safe;

        T Kp_rf_haa_pd_safe, Kd_rf_haa_pd_safe;
        T Kp_rf_hfe_pd_safe, Kd_rf_hfe_pd_safe;
        T Kp_rf_kfe_pd_safe, Kd_rf_kfe_pd_safe;

        T Kp_lh_haa_pd_safe, Kd_lh_haa_pd_safe;
        T Kp_lh_hfe_pd_safe, Kd_lh_hfe_pd_safe;
        T Kp_lh_kfe_pd_safe, Kd_lh_kfe_pd_safe;

        T Kp_rh_haa_pd_safe, Kd_rh_haa_pd_safe;
        T Kp_rh_hfe_pd_safe, Kd_rh_hfe_pd_safe;
        T Kp_rh_kfe_pd_safe, Kd_rh_kfe_pd_safe;
        

        UserParameter();
        ~UserParameter();
};






#endif