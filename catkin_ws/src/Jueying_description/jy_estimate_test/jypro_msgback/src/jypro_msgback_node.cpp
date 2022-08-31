#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <list>
#include <iostream>

#include "ros/ros.h"
#include "ros/node_handle.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include "time.h"

#include "external_type.h"

#define SERVER_PORT 1111
#define BUFF_LEN 2
#define SERVER_IP "192.168.1.120"
std::list<Data_feedback> data_buf;
int index_imu(0);
std::mutex my_mutex;
long long int joint_seq(0);
long long int imu_seq(0);

void udp_client(){
    int client_fd;
    struct sockaddr_in ser_addr;

    client_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(client_fd < 0){
        printf("create socket fail!\n");
        return;
    }

    memset(&ser_addr, 0 , sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    ser_addr.sin_port = htons(SERVER_PORT);

    socklen_t len;
    size_t buf_len;
    struct sockaddr_in src;
    Data_feedback buf;
    memset(&buf, 0, sizeof(Data_feedback));
    len = sizeof(ser_addr);
    buf_len = sizeof(buf);
    sendto(client_fd, &buf, buf_len, 0, (struct sockaddr*)&ser_addr, len);

    while (ros::ok())
    {   
        std::cout << "___udp_thread" << std::endl;
        recvfrom(client_fd, &buf, buf_len, 0, (struct sockaddr*)&src, &len);
        // std::cout << "server: " << std::endl;
        // std::cout << "leg_pos_lf: " << std::endl;
        // std::cout << buf.legState.fl_pos.value[0] << " " << buf.legState.fl_pos.value[1] <<  " " << buf.legState.fl_pos.value[2] << std::endl;

        my_mutex.lock();
        data_buf.push_back(buf);
        my_mutex.unlock();
    }

    close(client_fd);
}



int main(int argc, char* argv[]){
    std::thread readThread(udp_client);
    ros::init(argc, argv, "udp_node");
    ros::NodeHandle nh_;
    ros::Publisher jointState, imuState;
    sensor_msgs::JointState joint_msg;
    sensor_msgs::Imu imu_msg;
    ros::Rate rate(1000);
    ros::Time timer;
    bool get_data(false);

    jointState = nh_.advertise<sensor_msgs::JointState>("/joint_states",20);
    imuState = nh_.advertise<sensor_msgs::Imu>("/sensors/imu",20);

    Data_feedback state_data;

    while(nh_.ok()){
        std::cout << "ros_thread" << std::endl;
        if(!data_buf.empty()){
            state_data = data_buf.back();
            data_buf.pop_front();
            get_data = true;
        }
        if(get_data){
            std::cout << "Get Data Now!" << std::endl;
            joint_msg.name = {"LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE","LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE"};
            joint_msg.header.seq = joint_seq;
            joint_msg.header.frame_id = " ";
            joint_msg.header.stamp = timer.now();
            joint_msg.position.resize(12);
            joint_msg.velocity.resize(12);
            joint_msg.effort.resize(12);
            for(int i(0); i<3; i++){
                joint_msg.position[i]   = state_data.legState.fl_pos.value[i];
                joint_msg.position[i+3] = state_data.legState.fr_pos.value[i];
                joint_msg.position[i+6] = state_data.legState.hl_pos.value[i];
                joint_msg.position[i+9] = state_data.legState.hr_pos.value[i];

                joint_msg.velocity[i]   = state_data.legState.fl_vel.value[i];
                joint_msg.velocity[i+3] = state_data.legState.fr_vel.value[i];
                joint_msg.velocity[i+6] = state_data.legState.hl_vel.value[i];
                joint_msg.velocity[i+9] = state_data.legState.hr_vel.value[i];

                joint_msg.effort[i]   = state_data.legState.fl_torque.value[i];
                joint_msg.effort[i+3] = state_data.legState.fr_torque.value[i];
                joint_msg.effort[i+6] = state_data.legState.hl_torque.value[i];
                joint_msg.effort[i+9] = state_data.legState.hr_torque.value[i];
                
            }

            if(index_imu == 4){
                imu_msg.header.frame_id = "imu_link";
                imu_msg.header.seq = imu_seq;
                imu_msg.header.stamp = timer.now();
                
                Eigen::Quaternion<double> quat;
                quat = rpyTOquaternion(state_data.imuState.roll,
                                    state_data.imuState.pitch,
                                    state_data.imuState.yaw);
                imu_msg.orientation.w = quat.w();
                imu_msg.orientation.x = quat.x();
                imu_msg.orientation.y = quat.y();
                imu_msg.orientation.z = quat.z();

                imu_msg.angular_velocity.x = state_data.imuState.rol_vel;
                imu_msg.angular_velocity.y = state_data.imuState.pit_vel;
                imu_msg.angular_velocity.z = state_data.imuState.yaw_vel;

                imu_msg.linear_acceleration.x = state_data.imuState.acc_x;
                imu_msg.linear_acceleration.y = state_data.imuState.acc_y;
                imu_msg.linear_acceleration.z = state_data.imuState.acc_z;
                
                imuState.publish(imu_msg);
                imu_seq++;
                index_imu = 0;
            }

            jointState.publish(joint_msg);
            joint_seq++;
            index_imu++;
        }

        ros::spinOnce();
        rate.sleep();
    }

    readThread.join();

    return 0;
}