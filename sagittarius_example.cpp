/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>
#include <sdk_sagittarius_arm/modern_robotics.h>

int main(int argc, char** argv)
{
    float js[7];
    float joint_positions[7];
    ServoStruct servo_pose[7];
    sdk_sagittarius_arm::SagittariusArmReal sar("/dev/sagittarius", 1000000, 500, 5);
    sdk_sagittarius_arm::SagittariusArmIKinSpace ik(0.05, 0, 0);
    printf("Sagittarius driver is running\n");
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
        servo_pose[0].value = 0.55;
        servo_pose[1].id=2;
        servo_pose[1].value = 0.55;
        servo_pose[2].id=3;
        servo_pose[2].value = 0.55;    
        servo_pose[3].id=4;
        servo_pose[3].value = 1.55;           
        sar.SetServoRadianWithIndex(servo_pose,4);      //设置指定舵机的弧度
        sleep(1);
        sar.arm_set_gripper_linear_position(-0.068);    //设置夹爪的角度
        sleep(5);

        if(ik.getIKinTheta(0.35, 0, 0.0, 0, 0, 0, joint_positions))
        {
            sar.SetAllServoRadian(joint_positions);         //设置6个舵机的弧度
            sleep(5);
        }
        if(ik.getIKinTheta(0.3, 0, 0, 0, 90, 0, joint_positions))
        {
            sar.SetAllServoRadian(joint_positions);         //设置6个舵机的弧度
            sleep(5);
        }
        if(ik.getIKinTheta(0.3, 0, 0, 0, 45, 0, joint_positions))
        {
            sar.SetAllServoRadian(joint_positions);         //设置6个舵机的弧度
            sleep(5);
        }
        if(ik.getIKinTheta(0.2, 0.2, 0.2, 0, 0, 45, joint_positions))
        {
            sar.SetAllServoRadian(joint_positions);         //设置6个舵机的弧度
            sleep(5);
        }
        if(ik.getIKinTheta(0.3, 0, 0.3, 0, 0, 0, joint_positions))
        {
            sar.SetAllServoRadian(joint_positions);         //设置6个舵机的弧度
            sleep(5);
        }

    }
    return 0;
}






