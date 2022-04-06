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
Eigen::Quaternion<T> rpyTOquaternion(T roll, T pitch, T yaw){
    Eigen::Quaternion<T> quat;
    quat.x() = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    quat.y() = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    quat.z() = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    quat.w() = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    return quat;
}

// template<typename T>
// Eigen::Matrix<T, 3, 1> quaternionTOrpy(Eigen::Quaternion<T> q){
//     Eigen::Matrix<T, 3, 1> rpy;
//     // rpy[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(pow(q.x(), 2)+pow(q.y(), 2)));
//     // rpy[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));
//     // rpy[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2)+pow(q.z(), 2)));
	
//     return rpy;
// };