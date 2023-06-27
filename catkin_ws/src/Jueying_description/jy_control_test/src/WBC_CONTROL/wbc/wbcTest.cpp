#include "quadruped_dynamics_model.h"
#include "ros/ros.h"

#include "CoMAngularMotion.h"
#include "CoMLinearMotion.h"
#include "ContactForceLimits.h"
#include "ContactForceMin.h"
#include "EoMTask.h"
#include "NoContactMotion.h"
#include "TorqueLimits.h"
#include "SwingLegMotion.h"

#include "wbc.h"

#include "vector"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "jy_control_test_node");
    ros::NodeHandle n;
    QuadrupedDynamicsModel jueying(n, 0.001);

    FBModelState<double> state_;
    state_.bodyOrientation.w() = 1;
    state_.bodyOrientation.x() = 0;
    state_.bodyOrientation.y() = 0;
    state_.bodyOrientation.z() = 0;
    state_.bodyPosition << 0., 0.8, 0.5;
    state_.bodyVelocity << 0., 0.1, 0., 0., 0., 0.;
    state_.q_leg <<  0., -0.44, 1.44, 0., -0.443, 1.85, 0., -0.44, 1.44, 0., -0.443, 1.85;
    state_.qd_leg <<  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    state_.contact_state_ <<0, 1, 0, 1;

    jueying.setState(state_);

    DVec<double> foot_position, foot_vel, _N; 
    DMat<double> _H, _CJ, _JCOM, _JSF;
    jueying.massMatrix();
    jueying.nonlinearEffect();
    jueying.contactJacobian();
    jueying.CoM6DJacobian();
   
    foot_position = jueying.swingFootPosition(legID::LF);
    foot_vel = jueying.swingFootVelocity(legID::LF);
    _H =  jueying.getMassMatrix();
    _N = jueying.getNolinearEffect();
    _CJ = jueying.getContactJacobian();
    _JCOM = jueying.getCoM6DJacobian();
    _JSF =  jueying.swingFootJacobian(legID::LF);
        
    UserParameter<double> paramf;


    Task<double>* eom;
    eom = new EoMTask<double>(&jueying);
    eom->UpdateTask();
    ROS_INFO("___________________EOM TASK_____________________________________");
    ROS_INFO_STREAM("A_:   " << eom->get_A());
    ROS_INFO_STREAM("b_:   " << eom->get_b());
    ROS_INFO_STREAM("D_:   " << eom->get_D());
    ROS_INFO_STREAM("f_:   " << eom->get_f());


    Task<double>* nocontact;
    nocontact = new NoContactMotion<double> (&jueying);
    nocontact->UpdateTask();
    ROS_INFO("___________________NO CONTACT MOTION TASK_____________________________________");
    ROS_INFO_STREAM("A_:   " << nocontact->get_A());
    ROS_INFO_STREAM("b_:   " << nocontact->get_b());
    ROS_INFO_STREAM("D_:   " << nocontact->get_D());
    ROS_INFO_STREAM("f_:   " << nocontact->get_f());

    Task<double>* contactForce;
    contactForce = new ContactForceLimits<double>(&jueying);;
    contactForce->UpdateTask(paramf);
    ROS_INFO("___________________CONTACT FORCE LIMITS TASK_____________________________________");
    ROS_INFO_STREAM("A_:   " << contactForce->get_A());
    ROS_INFO_STREAM("b_:   " << contactForce->get_b());
    ROS_INFO_STREAM("D_:   " << contactForce->get_D());
    ROS_INFO_STREAM("f_:   " << contactForce->get_f());

    Task<double> * torqueLimit;
    torqueLimit = new TorqueLimits<double>(&jueying);
    torqueLimit->UpdateTask(paramf);
    ROS_INFO("___________________TORQUE LIMITS TASK_____________________________________");
    ROS_INFO_STREAM("A_:   " << torqueLimit->get_A());
    ROS_INFO_STREAM("b_:   " << torqueLimit->get_b());
    ROS_INFO_STREAM("D_:   " << torqueLimit->get_D());
    ROS_INFO_STREAM("f_:   " << torqueLimit->get_f());

    // Task<double>* comLinear;
    // comLinear = new CoMLinearMotion<double>(&jueying);
    //  Vec31<double> desPos, desVel, desAcc;
    // desPos << 1,2,3;
    // desVel << 4,5,6;
    // desAcc << 7,8,9;
    // comLinear->UpdateTask(desPos, desVel, desAcc, paramf);
    // ROS_INFO("___________________COM LINEAR MOTION TASK_____________________________________");
    // ROS_INFO_STREAM("A_:   " << comLinear->get_A());
    // ROS_INFO_STREAM("b_:   " << comLinear->get_b());
    // ROS_INFO_STREAM("D_:   " << comLinear->get_D());
    // ROS_INFO_STREAM("f_:   " << comLinear->get_f());
    
    // Task<double>* comAngular;
    // comAngular = new CoMAngularMotion<double>(&jueying);
    // Vec31<double> desAngle;
    // desAngle << 7.,8.,9.;
    // comAngular->UpdateTask(desAngle,paramf);
    // ROS_INFO("___________________COM ANGULAR MOTION TASK_____________________________________");
    // ROS_INFO_STREAM("A_:   " << comAngular->get_A());
    // ROS_INFO_STREAM("b_:   " << comAngular->get_b());
    // ROS_INFO_STREAM("D_:   " << comAngular->get_D());
    // ROS_INFO_STREAM("f_:   " << comAngular->get_f());

    // Task<double>* swingLeg;
    // swingLeg = new SwingLegMotion<double> (&jueying);
    // desPos << 1,2,3;
    // desVel << 4,5,6;
    // desAcc << 7,8,9;
    // swingLeg->UpdateTask(desPos, desVel, desAcc, paramf);
    // ROS_INFO("___________________SWING LEG MOTION TASK_____________________________________");
    // ROS_INFO_STREAM("A_:   " << swingLeg->get_A());
    // ROS_INFO_STREAM("b_:   " << swingLeg->get_b());
    // ROS_INFO_STREAM("D_:   " << swingLeg->get_D());
    // ROS_INFO_STREAM("f_:   " << swingLeg->get_f());

    // Task<double>* contactMin;
    // contactMin = new ContactForceMin<double>(&jueying);
    // contactMin->UpdateTask();
    // ROS_INFO("___________________CONTACT FORCE MIN TASK_____________________________________");
    // ROS_INFO_STREAM("A_:   " << contactMin->get_A());
    // ROS_INFO_STREAM("b_:   " << contactMin->get_b());
    // ROS_INFO_STREAM("D_:   " << contactMin->get_D());
    // ROS_INFO_STREAM("f_:   " << contactMin->get_f());

    
    std::vector<Task<double>* > task_lists_;
    task_lists_.push_back(eom);
    task_lists_.push_back(nocontact);
    task_lists_.push_back(contactForce);
    task_lists_.push_back(torqueLimit);
    

    // WBC<double> wholeBodyControl(&task_lists_, paramf);
    // wholeBodyControl._SetOptimizationSizeTEST(1); 
    // ROS_INFO("___________________WBC RESULT_____________________________________");
    // ROS_INFO_STREAM("dim_n_:   " << wholeBodyControl.dim_n_);    
    // ROS_INFO_STREAM("dim_config_:   "<<wholeBodyControl.dim_confi_);
    // ROS_INFO_STREAM("dim_contact_:   "<<wholeBodyControl.dim_conta_);
    // ROS_INFO_STREAM("dim_opt_:   " << wholeBodyControl.dim_opt_);
    // ROS_INFO_STREAM("dim_nv_now_:   " <<  wholeBodyControl.dim_nv_now_);
    // ROS_INFO_STREAM("dim_Dbar_R_:   " <<  wholeBodyControl.dim_Dbar_R_);

    // WBC<double> wholeBodyControl(&task_lists_, paramf);
    // wholeBodyControl._SetOptimizationSizeTEST(1); 
    // wholeBodyControl._Update_CostFuntionTEST(1); 
    // ROS_INFO("___________________WBC RESULT_____________________________________");
    // // ROS_INFO_STREAM("dim_n_:   " << wholeBodyControl.dim_n_);    
    // // ROS_INFO_STREAM("dim_config_:   "<<wholeBodyControl.dim_confi_);
    // // ROS_INFO_STREAM("dim_contact_:   "<<wholeBodyControl.dim_conta_);
    // // ROS_INFO_STREAM("dim_opt_:   " << wholeBodyControl.dim_opt_);
    // // ROS_INFO_STREAM("dim_nv_now_:   " <<  wholeBodyControl.dim_nv_now_);
    // // ROS_INFO_STREAM("dim_Dbar_R_:   " <<  wholeBodyControl.dim_Dbar_R_);
    // ROS_INFO_STREAM("dim_eq_pre:   " <<  wholeBodyControl.dim_eq_pre_);

    // WBC<double> wholeBodyControl(&task_lists_, paramf);
    // wholeBodyControl._SetOptimizationSizeTEST(1); 
    // wholeBodyControl._Update_InEqConstraintTEST(1); 
    // ROS_INFO("___________________WBC RESULT_____________________________________");
    // // ROS_INFO_STREAM("dim_n_:   " << wholeBodyControl.dim_n_);    
    // // ROS_INFO_STREAM("dim_config_:   "<<wholeBodyControl.dim_confi_);
    // // ROS_INFO_STREAM("dim_contact_:   "<<wholeBodyControl.dim_conta_);
    // // ROS_INFO_STREAM("dim_opt_:   " << wholeBodyControl.dim_opt_);
    // // ROS_INFO_STREAM("dim_nv_now_:   " <<  wholeBodyControl.dim_nv_now_);
    // // ROS_INFO_STREAM("dim_Dbar_R_:   " <<  wholeBodyControl.dim_Dbar_R_);
    // ROS_INFO_STREAM("dim_eq_pre:   " <<  wholeBodyControl.dim_eq_pre_);

    WBC<double>* wholeBodyControl;
    wholeBodyControl = new WBC<double>(&task_lists_, paramf);
    DVec<double> CMD;
    wholeBodyControl->UpdateSetting(_H.cast<double>(), _CJ.cast<double>(), _N.cast<double>());
    wholeBodyControl->MakeTorque(CMD);

    ROS_INFO("___________________WBC RESULT_____________________________________");

    //ROS_INFO_STREAM("dim_eq_pre:   " <<  wholeBodyControl.dim_eq_pre_);

    delete eom;
    delete nocontact;
    delete contactForce;
    delete torqueLimit;
    delete wholeBodyControl;

    return 0;
}