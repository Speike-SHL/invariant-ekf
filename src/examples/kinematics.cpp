/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   kinematics.cpp
 *  @author Ross Hartley
 *  @brief  Example of invariant filtering for contact-aided inertial navigation
 *  @date   September 25, 2018
 **/

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "InEKF.h"
#include <boost/filesystem.hpp>

#define DT_MIN 1e-6
#define DT_MAX 1

using namespace std;
using namespace inekf;

// 将字符串转换为double类型
double stod98(const std::string &s) {
    return atof(s.c_str());
}

// 将字符串转换为int类型
int stoi98(const std::string &s) {
    return atoi(s.c_str());
}

int main() {
    boost::filesystem::path path = boost::filesystem::current_path();
    std::cout << "Current path is : " << path << std::endl;
    //  ---- Initialize invariant extended Kalman filter ----- //
    RobotState initial_state; 

    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    // HACK 为什么不是[1 0 0; 0 0 -1; 0 1 0]
    R0 << 1, 0, 0, // initial orientation
          0, -1, 0, // IMU frame is rotated 90deg about the x-axis
          0, 0, -1; 
    v0 << 0,0,0; // initial velocity
    p0 << 0,0,0; // initial position
    bg0 << 0,0,0; // initial gyroscope bias
    ba0 << 0,0,0; // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize state covariance
    NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.01);
    noise_params.setAccelerometerNoise(0.1);
    noise_params.setGyroscopeBiasNoise(0.00001);
    noise_params.setAccelerometerBiasNoise(0.0001);
    noise_params.setContactNoise(0.01);

    // Initialize filter
    InEKF filter(initial_state, noise_params);
    cout << "Noise parameters are initialized to: \n";
    cout << filter.getNoiseParams() << endl;    //在NoiseParams中重载了运算符<<
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;

    // Open data file
    ifstream infile("./src/data/imu_kinematic_measurements.txt");
    string line;
    Eigen::Matrix<double,6,1> imu_measurement = Eigen::Matrix<double,6,1>::Zero();  //imu测量值
    Eigen::Matrix<double,6,1> imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();  //上一次的imu测量值
    double t = 0;
    double t_prev = 0;

    // ---- Loop through data file and read in measurements line by line ---- //
    while (getline(infile, line)){
        vector<string> measurement;
        boost::split(measurement,line,boost::is_any_of(" "));  //将一行按空格分割后保存到measurement中
        // Handle measurements
        // IMU time w*3 a*3
        if (measurement[0].compare("IMU")==0){  //读取储存imu行
            cout << "Received IMU Data, propagating state\n";
            assert((measurement.size()-2) == 6);
            t = atof(measurement[1].c_str()); 
            // Read in IMU data
            imu_measurement << stod98(measurement[2]), 
                               stod98(measurement[3]), 
                               stod98(measurement[4]),
                               stod98(measurement[5]),
                               stod98(measurement[6]),
                               stod98(measurement[7]);

            // Propagate using IMU data
            double dt = t - t_prev;
            if (dt > DT_MIN && dt < DT_MAX) {
                filter.Propagate(imu_measurement_prev, dt);  // HACK用上次的IMU数据预测?为什么先预测后更新contact
            }

        }
        // CONTACT time 腿id 状态 腿id 状态
        else if (measurement[0].compare("CONTACT")==0){
            cout << "Received CONTACT Data, setting filter's contact state\n";
            assert((measurement.size()-2)%2 == 0);
            vector<pair<int,bool> > contacts;   //储存腿id和腿状态
            int id;
            bool indicator;
            t = stod98(measurement[1]); 
            // Read in contact data
            for (int i=2; i<measurement.size(); i+=2) {
                id = stoi98(measurement[i]);
                indicator = bool(stod98(measurement[i+1]));
                contacts.push_back(pair<int,bool> (id, indicator));
            } 
            // Set filter's contact state
            filter.setContacts(contacts);//HACK 为什么在预测后设置contact
        }
        // KINEMATIC time 腿id Quaternion*4 p*3 协方差矩阵6x6=36 腿id Quaternion*4 p*3 协方差矩阵6x6=36
        else if (measurement[0].compare("KINEMATIC")==0){
            cout << "Received KINEMATIC observation, correcting state\n";  
            assert((measurement.size()-2)%44 == 0);
            int id;
            Eigen::Quaternion<double> q;    // 读取四元数
            Eigen::Vector3d p;              // 读取位移
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity(); //组合成变换矩阵
            Eigen::Matrix<double,6,6> covariance;
            vectorKinematics measured_kinematics;
            t = stod98(measurement[1]); 
            // Read in kinematic data
            for (int i=2; i<measurement.size(); i+=44) {
                id = stoi98(measurement[i]); 
                q = Eigen::Quaternion<double> (stod98(measurement[i+1]),stod98(measurement[i+2]),stod98(measurement[i+3]),stod98(measurement[i+4]));
                q.normalize();
                p << stod98(measurement[i+5]),stod98(measurement[i+6]),stod98(measurement[i+7]);
                pose.block<3,3>(0,0) = q.toRotationMatrix();
                pose.block<3,1>(0,3) = p;
                for (int j=0; j<6; ++j) {   //6x6协方差矩阵, 对角阵, 右下3x3的范围为J_p(\alpha)*Cov(w_t^\alpha)*J_p(\alpha).transpose
                    for (int k=0; k<6; ++k) {
                        covariance(j,k) = stod98(measurement[i+8 + j*6+k]);
                    }
                }
                Kinematics frame(id, pose, covariance); // 组合成一个正向运动学对象
                measured_kinematics.push_back(frame);
            }
            // Correct state using kinematic measurements
            filter.CorrectKinematics(measured_kinematics);  //使用正向运动学进行更新
        }

        // Store previous timestamp
        t_prev = t;
        imu_measurement_prev = imu_measurement;
    }

    // Print final state
    cout << filter.getState() << endl;
    return 0;
}
