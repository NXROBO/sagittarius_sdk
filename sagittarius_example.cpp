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
    float joint_positions[7];
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
        sleep(1);
        sar.arm_set_gripper_linear_position(0.0);      //设置夹爪的角度
        sar.GetCurrentJointStatus(js);                 //获取当前各个舵机的弧度到js
        //printf("----%f %f %f %f %f %f %f \n",js[0], js[1], js[2], js[3], js[4], js[5], js[6]);
        joint_positions[0] = 0;
        joint_positions[1] = 0;
        joint_positions[2] = 0;
        joint_positions[3] = 0;
        joint_positions[4] = 0;
        joint_positions[5] = 1.576;
        joint_positions[6] = 0;
        sar.SetAllServoRadian(joint_positions);         //设置6个舵机的弧度
        sleep(1);

        joint_positions[5] = 0;
        joint_positions[6] = 0;
        sar.SetAllServoRadian(joint_positions);         //设置6个舵机的弧度
        sleep(1);
        servo_pose[0].id=1;
        servo_pose[0].value = 0.45;
        servo_pose[1].id=2;
        servo_pose[1].value = 0.45;
        servo_pose[2].id=3;
        servo_pose[2].value = 0.55;    
        servo_pose[3].id=4;
        servo_pose[3].value = 1.55;           
        sar.SetServoRadianWithIndex(servo_pose,4);      //设置指定舵机的弧度
        sleep(1);
        sar.arm_set_gripper_linear_position(-0.068);    //设置夹爪的角度
        sleep(5);

        // 获取末端在目标点上时每个舵机的角度，使用XYZ位置与欧拉角姿态
        if(sgr_kinematics.getIKinThetaEuler(0.3, 0, 0.0, 0, 0, 0, joint_positions))
        {
            // 这个位置 IK 运算器能找到结果，但角度在舵机运行范围外，所以结果不执行，并打印错误信息
            sar.SetAllServoRadian(joint_positions);                 //设置6个舵机的弧度
            sleep(5);
        }

        // 获取末端在目标点上时每个舵机的角度，使用XYZ位置与欧拉角姿态
        if(sgr_kinematics.getIKinThetaEuler(0.3, 0, 0.1, 0, 45, 0, joint_positions)) 
        {
            sar.SetAllServoRadian(joint_positions);                 //设置6个舵机的弧度
            sleep(5);
            
            float xyz[3] = {0}, euler[3] = {0};
            log_print(LOG_TYPE_INFO, "current XYZ and Euler:\n");
            sar.GetCurrentJointStatus(js);                          // 获取当前角度
            sgr_kinematics.getFKinEuler(js, xyz, euler);            // 对 js 进行正向运动学，返回位置和欧拉角姿态
            log_print(LOG_TYPE_INFO, "x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f\n", xyz[0], xyz[1], xyz[2], euler[0], euler[1], euler[2]);
            sleep(2);
        }

        // 获取末端在目标点上时每个舵机的角度，使用XYZ位置与四元数姿态(roll=0, pitch=0.785rad, yaw=0)
        if(sgr_kinematics.getIKinThetaQuaternion(0.3, 0, 0.1, 0, 0.7068252, 0, 0.7073883, joint_positions))
        {
            sar.SetAllServoRadian(joint_positions);                 //设置6个舵机的弧度
            sleep(5);
        }

        // 获取末端在目标点上时每个舵机的角度，使用XYZ位置与欧拉角姿态
        if(sgr_kinematics.getIKinThetaEuler(0.2, 0.2, 0.2, 0, 0, 45, joint_positions))
        {
            sar.SetAllServoRadian(joint_positions);                 //设置6个舵机的弧度
            sleep(5);
        }

        // 获取末端在目标点上时每个舵机的角度，使用XYZ位置与欧拉角姿态
        if(sgr_kinematics.getIKinThetaEuler(0.3, 0, 0.3, 0, 0, 0, joint_positions))
        {
            sar.SetAllServoRadian(joint_positions);                 //设置6个舵机的弧度
            sleep(5);
        }

    }
    return 0;
}






