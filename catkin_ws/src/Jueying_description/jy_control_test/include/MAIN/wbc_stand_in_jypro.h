#ifndef INTERFACE_EXTERNAL_TYPE_H_
#define INTERFACE_EXTERNAL_TYPE_H_

#include <cmath>

using namespace std;

const double kPI = 3.1415926535898;
const double kDegree2Radian = kPI / 180;

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

struct DataGlobal
{
    bool stand_up_down_flag_record;
    int state;
    double state_initial_time;

    OneTypeLegData fl_initial_angle;
    OneTypeLegData fr_initial_angle;
    OneTypeLegData hl_initial_angle;
    OneTypeLegData hr_initial_angle;
};

void SwingToAngle(const OneTypeLegData initial_angle, const OneTypeLegData final_angle, const OneTypeLegData now_angle, const OneTypeLegData now_vel, const double total_time, const double run_time, const double cycle_time_,
				  OneTypeLegData *angle_output, OneTypeLegData *vel_output, OneTypeLegData *torque_output);
void KeepAngle(const OneTypeLegData goal_angle, const OneTypeLegData now_angle, const OneTypeLegData now_vel, OneTypeLegData *torque_output);
void CubicSpline(double init_pos, double init_vel, double goal_pos, double goal_vel, double run_time, double cycle_time, double total_time, double &sub_goal_pos, double &sub_goal_pos_next);
// void AddToScope(const double data, const int i, scope_output *scope_output);
void GetAbsMin(const double reference, double *own);

void SwingToAngle(const OneTypeLegData initial_angle, const OneTypeLegData final_angle, const OneTypeLegData now_angle, const OneTypeLegData now_vel, const double total_time, const double run_time, const double cycle_time_,
				  OneTypeLegData *angle_output, OneTypeLegData *vel_output, OneTypeLegData *torque_output)
{
	OneTypeLegData goal_angle;
	OneTypeLegData goal_angle_next;
	OneTypeLegData goal_ang_vel;

	for (int i = 0; i<3; i++) {
		CubicSpline(initial_angle.value[i], 0, (double)final_angle.value[i], 0,
			run_time, cycle_time_, total_time, goal_angle.value[i], goal_angle_next.value[i]);
		goal_ang_vel.value[i] = (goal_angle_next.value[i] - goal_angle.value[i]) / cycle_time_;
		angle_output->value[i] = goal_angle.value[i];
		vel_output->value[i] = goal_ang_vel.value[i];
	}

	// PD control
	torque_output->value[0] = 200 * (goal_angle.value[0] - now_angle.value[0]) + 2 * (goal_ang_vel.value[0] - now_vel.value[0]);
	torque_output->value[1] = 200 * (goal_angle.value[1] - now_angle.value[1]) + 2 * (goal_ang_vel.value[1] - now_vel.value[1]);
	torque_output->value[2] = 300 * (goal_angle.value[2] - now_angle.value[2]) + 3 * (goal_ang_vel.value[2] - now_vel.value[2]);
}

void KeepAngle(const OneTypeLegData goal_angle, const OneTypeLegData now_angle, const OneTypeLegData now_vel, OneTypeLegData *torque_output)
{
	torque_output->value[0] = 200 * (goal_angle.value[0] - now_angle.value[0]) + 2 * (0 - now_vel.value[0]);
	torque_output->value[1] = 200 * (goal_angle.value[1] - now_angle.value[1]) + 2 * (0 - now_vel.value[1]);
	torque_output->value[2] = 300 * (goal_angle.value[2] - now_angle.value[2]) + 3 * (0 - now_vel.value[2]);
}

void CubicSpline(double init_pos, double init_vel, double goal_pos, double goal_vel, double run_time, double cycle_time, double total_time, double &sub_goal_pos, double &sub_goal_pos_next)
{
	double a, b, c, d;
	d = init_pos;
	c = init_vel;
	a = (goal_vel * total_time - 2 * goal_pos + init_vel * total_time + 2 * init_pos) / pow(total_time, 3);
	b = (3 * goal_pos - goal_vel * total_time - 2 * init_vel*total_time - 3 * init_pos) / pow(total_time, 2);

	if (run_time > total_time)
		run_time = total_time;
	sub_goal_pos = a * pow(run_time, 3) + b * pow(run_time, 2) + c * run_time + d;

	if (run_time + cycle_time > total_time)
		run_time = total_time - cycle_time;
	sub_goal_pos_next = a * pow(run_time + cycle_time, 3) + b * pow(run_time + cycle_time, 2) + c * (run_time + cycle_time) + d;
}

// void AddToScope(const double data, const int i, scope_output *scope_output)
// {
// 	(*scope_output)[i] = data;
// }

void GetAbsMin(const double reference, double *own)
{
	if (abs(*own) > abs(reference)) {
		if (*own > 0) {
			*own = abs(reference);
		} else {
			*own = -abs(reference);
		}
	}
}


#endif