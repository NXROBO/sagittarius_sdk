#ifndef SDK_ARM_COMMANDS_CONSTANTS_
#define SDK_ARM_COMMANDS_CONSTANTS_

#define PI 3.141592653589793
#define TYPE_REQUEST_MESSAGE 0x01
#define CMD_CONTROL_END_ACTION 0x03
#define CMD_CONTROL_ID_DEGREE 0x04
#define CMD_CONTROL_ALL_DEGREE 0x05

#define CMD_CONTROL_LOCK_OR_FREE 0x08
#define CMD_CONTROL_ALL_DEGREE_CB 0x0B //插补控制命令
#define CMD_CONTROL_ALL_DEGREE_AND_DIFF_TIME 0x0C
#define CMD_SET_SERVO_VEL 0x0D    //设置舵机的插补速度
#define CMD_SET_SERVO_ACC 0x0E    //设置舵机的加速度
#define CMD_SET_SERVO_TORQUE 0x0F //设置舵机的扭矩厌大小
#define CMD_GET_SERVO_RT_INFO 0x10 //获取舵机实时信息

// 关节角度限制, 数组下标 0~5 代表关节编号 1~6, 单位是弧度
#define JOINT_LOWER_LIMITS {-2, -1.57, -1.48, -2.9, -1.8, -3.1} // 最小角度
#define JOINT_UPPER_LIMITS {2, 1.4, 1.8, 2.9, 1.6, 3.1} // 最大角度

#endif // SDK_ARM_COMMANDS_CONSTANTS_
