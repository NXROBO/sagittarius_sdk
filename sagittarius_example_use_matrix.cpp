/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
#include <iostream>
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_log.h>
#include <sdk_sagittarius_arm/modern_robotics.h>

int main(int argc, char** argv)
{
    float js[7];
    float joint_positions[7] = {0};
    int torque[7] = {300, 300, 300, 300, 300, 300, 300}; // 舵机扭矩的值，下标0~6对应舵机编号1~7
    ServoStruct servo_pose[7];

    log_set_level(3); // 设置日志级别为 3，如果不调用设置时，级别默认为 3

    // 连接机械臂
    // 参数1："/dev/ttyACM0" 是机械臂连接Linux后，设备描述文件的路径
    // 参数2：1000000 为波特率
    // 参数3：500 为最高运行速度
    // 参数4：5 为加速度
    sdk_sagittarius_arm::SagittariusArmReal sar("/dev/ttyACM0", 1000000, 500, 5);

    // 初始化机械臂 IK 运算器
    // 机械臂默认的效应器在 7 号舵机的舵盘中心。
    // 参数x、y、z 作用是对效应器进行偏移，之后的运算都使用偏移后的效应器
    sdk_sagittarius_arm::SagittariusArmKinematics sgr_kinematics(0, 0, 0);

    // sar.SetServoTorque(torque); // 扭力设置
    log_print(LOG_TYPE_INFO, "Sagittarius driver is running\n"); // 输出常规信息
    while(1)
    {
        joint_positions[0] = 0;
        joint_positions[1] = 0;
        joint_positions[2] = 0;
        joint_positions[3] = 0;
        joint_positions[4] = 0;
        joint_positions[5] = 0;
        sar.SetAllServoRadian(joint_positions);         //设置6个舵机的弧度
        sleep(3);

        // 获取旋转矩阵，使用欧拉角转换，静态RPY，yaw = 45 degree
        Eigen::AngleAxisd yawAxis = Eigen::AngleAxisd(-45 * M_PI / 180, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchAxis = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rollAxis = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d R = yawAxis.matrix() * pitchAxis.matrix() * rollAxis.matrix();
        
        // 获取变换矩阵
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();        // 变换矩阵
        T.rotate(R);                                                // 输入旋转矩阵
        T.pretranslate(Eigen::Vector3d(0.2, -0.2, 0.2));            // 平移向量, x=0.2 y=0.2 z=0.2
        log_print(LOG_TYPE_INFO, "target matrix:\n");
        std::cout << T.matrix() << std::endl;

        // 以变换矩阵作为末端效应器目标点，逆向运动学运算
        if(sgr_kinematics.getIKinThetaMatrix(T.matrix(), joint_positions)) 
        {
            sar.SetAllServoRadian(joint_positions);                 //设置6个舵机的弧度
            sleep(5);

            // 4x4单位矩阵
            Eigen::MatrixXd T_EE = Eigen::MatrixXd::Identity(4, 4);
            sar.GetCurrentJointStatus(js);                          // 获取当前角度
            sgr_kinematics.getFKinMatrix(js, T_EE);                 // 对 js 进行正向运动学，返回矩阵
            log_print(LOG_TYPE_INFO, "current matrix:\n");
            std::cout << T_EE << std::endl;
            sleep(2);
        }
    }
    return 0;
}






