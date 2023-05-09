#ifndef SDK_SAGITTARIUS_ARM_REAL_
#define SDK_SAGITTARIUS_ARM_REAL_
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <vector>
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.h>
#include <sdk_sagittarius_arm/modern_robotics.h>

#define JOINT_NUM 6


namespace sdk_sagittarius_arm
{
    class SagittariusArmReal
    {
    public:
        /// @brief SagittariusArmReal 初始化类
        /// @param strSerialName - std::string 对应机械臂的串口名，默认是“/dev/sagittarius”
        /// @param Baudrate - int 波特率，默认是1000000       
        /// @param vel - int 舵机的最高运行速度，对应7个舵机的速度0～4096， 建议设置在1000左右  
        /// @param acc - int 舵机的加速度， 对应舵机的加速度0～254。建议设置在10以内                            
        SagittariusArmReal(std::string strSerialName, int Baudrate, int vel, int acc);
        virtual ~SagittariusArmReal();

        /// @brief SetFreeAfterDestructor 设置对象销毁时是否释放舵机
        /// @param value - true为释放, false为不释放
        void SetFreeAfterDestructor(bool value);

        /// @brief CheckUpperLower 检测输入的角度是否在允许的范围内
        /// @param js「」 - js[0]～js[5]对应1～6号舵机的弧度值。请一次性按顺序传入6个舵机值
        bool CheckUpperLower(float js[]);

        /// @brief CheckUpperLowerWithIndex 检测输入的角度是否在允许的范围内
        /// @param sv「」 - sv[].id为舵机号，sv[].value为对应舵机的弧度值。对应的舵机的数组可以不按顺序排列，但最多六个舵机。
        /// @param num - num为要控制的舵机的个数。最多六个舵机。
        bool CheckUpperLowerWithIndex(ServoStruct sv[], int num);


        /// @brief arm_set_gripper_linear_position 设置夹爪的位置
        /// @param dist - 夹爪的距离值 -0.068~0.0(其中-0.068表示全关闭，0.0表示全开)
        void arm_set_gripper_linear_position(const float dist);

        /// @brief SetAllServoRadian 控制舵机运行到相应的弧度
        /// @param joint_positions「」 - joint_positions「0」~joint_positions「5」分别对应舵机1～6的弧定，一次性按顺序输入弧度
        void SetAllServoRadian(float joint_positions[]);

        /// @brief GetCurrentJointStatus 获取当前各个秀
        /// @param js「」 - js「」按顺序存放每个舵机当前弧度的大小。一次性按顺序获取7个舵机       
        bool GetCurrentJointStatus(float js[]);

        /// @brief SetServoRadianWithIndex 设定指定舵机的弧度。
        /// @param sv「」 - sv[].id为舵机号，sv[].value为对应舵机的弧度值。对应的舵机的数组可以不按顺序排列，但最多六个舵机。
        /// @param num - num为要控制的舵机的个数。最多六个舵机。       
        void SetServoRadianWithIndex(ServoStruct sv[], int num);

        /// @brief ControlTorque 设置舵机锁舵或者释放
        /// @param msg - 字符串，"free" 或者 "lock"
        void ControlTorque(std::string msg);

        /// @brief GetServoInfo 获取指定舵机的实时状态
        /// @param id - 指定舵机的舵机号
        /// @param info_arr - 2字节整型的数组，成功获取实时状态后，[速度(50step/s), 占空比(%), 电压(V), 电流(mA)]共4个状态会按顺序保存这个数组中
        /// @param timeout_ms - 读取超时的时间，超过时间就返回读取失败，单位是ms，建议设置值大于100ms，默认500ms
        bool GetServoInfo(unsigned char id, int16_t info_arr[], int timeout_ms=500);

        /// @brief SetServoAcceleration 设置所有舵机的加速度
        /// @param arm_acc - 对应舵机的加速度0～254。建议设置在10以内        
        bool SetServoAcceleration(int arm_acc);

        /// @brief SetServoVelocity 设置所有舵机的最高速度
        /// @param arm_vel - 对应舵机的速度0～4096， 建议设置在1000左右       
        bool SetServoVelocity(int arm_vel);

        /// @brief SetServoTorque 设置所有舵机的扭距
        /// @param arm_torque「」 - arm_torque「」按顺序存放每个舵机扭矩的大小。一次性按顺序存放7个舵机    
        bool SetServoTorque(int arm_torque[]);

    public:
        float lower_joint_limits[6];
        float upper_joint_limits[6];

    private:
        bool torque_status;
        bool free_after_destructor;
        sdk_sagittarius_arm::CSDarmCommon *pSDKarm;
        float angle[JOINT_NUM];
        /// @brief arm_calculate_gripper_degree_position 距离转换成角度
        /// @param dist - 夹爪的距离值     
        short arm_calculate_gripper_degree_position(const float dist);

    };
    class SagittariusArmKinematics
    {
        public:
            /// @brief SagittariusArmKinematics 初始化类
            /// @param x - float 末端在 x 轴的偏移, 单位为米
            /// @param y - float 末端在 y 轴的偏移, 单位为米
            /// @param z - float 末端在 z 轴的偏移, 单位为米
            SagittariusArmKinematics(float x = 0, float y = 0, float z = 0);
            virtual ~SagittariusArmKinematics();

            /// @brief getIKinThetaMatrix 给定末端位姿的矩阵，运算出机械臂各个关节的角度
            /// @param M - 末端在世界坐标上表达的矩阵
            /// @param theta_result「」- 运算出关节的解会存储在这个数组中
            /// @param eomg - 姿态的精度
            /// @param ev - 位置的精度
            bool getIKinThetaMatrix(const Eigen::MatrixXd& M_EE, float theta_result[], double eomg = 0.001, double ev = 0.001);

            /// @brief getIKinThetaEuler 给定末端位置和以欧拉角(静态RPY)表达的姿态，运算出机械臂各个关节的角度
            /// @param x - 末端在 x 轴上的值
            /// @param y - 末端在 y 轴上的值
            /// @param z - 末端在 z 轴上的值
            /// @param roll - 末端欧拉角 roll, 角度制
            /// @param pitch - 末端欧拉角 pitch, 角度制
            /// @param yaw - 末端欧拉角 yaw, 角度制
            /// @param theta_result「」- 运算出关节的解会存储在这个数组中
            /// @param eomg - 姿态的精度
            /// @param ev - 位置的精度
            bool getIKinThetaEuler(float x, float y, float z, float roll, float pitch, float yaw, float theta_result[], double eomg = 0.001, double ev = 0.001);

            /// @brief getIKinThetaQuaternion 给定末端位置和以四元数表达的姿态，运算出机械臂各个关节的角度
            /// @param x - 末端在 x 轴上的值
            /// @param y - 末端在 y 轴上的值
            /// @param z - 末端在 z 轴上的值
            /// @param ox - 末端四元数的 x 值
            /// @param oy - 末端四元数的 y 值
            /// @param oz - 末端四元数的 z 值
            /// @param ow - 末端四元数的 w 值
            /// @param theta_result「」- 运算出关节的解会存储在这个数组中
            /// @param eomg - 姿态的精度
            /// @param ev - 位置的精度
            bool getIKinThetaQuaternion(float x, float y, float z, float ox, float oy, float oz, float ow, float theta_result[], double eomg = 0.001, double ev = 0.001);

            /// @brief getFKinMatrix 给定6个关节的角度，运算出机械臂末端的矩阵
            /// @param theta「」 - 6个关节的角度
            /// @param M - 运算出的矩阵会存储在这个对象中
            bool getFKinMatrix(float theta[], Eigen::MatrixXd& M_EE);

            /// @brief getFKinMatrix 给定6个关节的角度，运算出机械臂末端的位置与欧拉角
            /// @param theta「」 - 6个关节的角度
            /// @param xyz - 运算出的位置会存储在这个对象中
            /// @param euler - 运算出的姿态会存储在这个对象中
            bool getFKinEuler(float theta[], float xyz[], float euler[]);

            /// @brief getFKinMatrix 给定6个关节的角度，运算出机械臂末端的位置与四元数
            /// @param theta「」 - 6个关节的角度
            /// @param xyz - 运算出的位置会存储在这个对象中
            /// @param quaternion - 运算出的姿态会存储在这个对象中
            bool getFKinQuaternion(float theta[], float xyz[], float quaternion[]);

        public:
            float lower_joint_limits[6];
            float upper_joint_limits[6];

        private:
            Eigen::MatrixXd Slist;
            Eigen::Matrix4d M;
    };

} // sdk_sagittarius_arm

#endif // SDK_SAGITTARIUS_ARM_REAL_
