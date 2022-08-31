#include <eigen3/Eigen/Geometry>

typedef struct
{
	double value[3];
} OneTypeLegData;

typedef struct
{
	OneTypeLegData fl_pos;
	OneTypeLegData fr_pos;
	OneTypeLegData hl_pos;
	OneTypeLegData hr_pos;
	OneTypeLegData fl_vel;
	OneTypeLegData fr_vel;
	OneTypeLegData hl_vel;
	OneTypeLegData hr_vel;
	OneTypeLegData fl_torque;
	OneTypeLegData fr_torque;
	OneTypeLegData hl_torque;
	OneTypeLegData hr_torque;
} DataLegs;

typedef struct
{
	double roll; double pitch; double yaw;
	double rol_vel; double pit_vel; double yaw_vel;
	double acc_x; double acc_y; double acc_z;
} DataGyro;

struct Data_feedback{
	double time_stamp;
	DataLegs legState;
	DataGyro imuState;
};

template<typename T>
Eigen::Matrix<T,3,3> rpyTORotateMat(T roll, T pitch, T yaw){
    Eigen::Matrix<T,3,3> RotateMatrix, R_roll, R_pitch, R_yaw;
    R_roll <<  1., 0., 0., 
               0., cos(roll), -sin(roll),
               0., sin(roll), cos(roll);
    R_pitch << cos(pitch), 0, sin(pitch),
               0., 1., 0.,
               -sin(pitch), 0., cos(pitch);
    R_yaw << cos(yaw), -sin(yaw), 0.,
             sin(yaw), cos(yaw), 0.,
             0., 0., 1.;
    RotateMatrix = R_yaw * R_pitch * R_roll;
    return RotateMatrix;
}

template<typename T>
Eigen::Quaternion<T> rpyTOquaternion(T roll, T pitch, T yaw){
    Eigen::Quaternion<T> q(rpyTORotateMat(roll,pitch,yaw));
    return q;
}

// template<typename T>
// Eigen::Matrix<T, 3, 1> quaternionTOrpy(Eigen::Quaternion<T> q){
//     Eigen::Matrix<T, 3, 1> rpy;
//     // rpy[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(pow(q.x(), 2)+pow(q.y(), 2)));
//     // rpy[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));
//     // rpy[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2)+pow(q.z(), 2)));
	
//     return rpy;
// };