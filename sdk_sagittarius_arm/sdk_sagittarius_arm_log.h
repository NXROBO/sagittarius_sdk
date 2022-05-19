/*
 * @Descripttion: ��־
 * @version: 1.00
 * @Author: ShuDong.Hong@nxrobo.com
 * @Company: NXROBO (���ڴ���δ�����������޹�˾)
 * @Date: 2022-03-29 11:10:52
 * @LastEditors: ShuDong.Hong@nxrobo.com
 * @LastEditTime: 2022-03-30 11:49:33
 */
#ifndef SDK_ARM_LOG_
#define SDK_ARM_LOG_
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_constants.h>


// ��־��Ϣ�ļ���
#define LOG_TYPE_DEBUG 4 // ������Ϣ, ���ڵ���
#define LOG_TYPE_INFO 3  // ������Ϣ
#define LOG_TYPE_WARN 2  // һ�㾯��, �ɼ���ִ�е��޷���֤����������
#define LOG_TYPE_ERROR 1 // ���ش���, �޷�����ִ��

/// @brief log_print ���ָ���������־����һ������ʱ��־�����ǩ���������ʹ���� printf һ��
/// @param level - ��־�����ǩ���ֱ��� LOG_TYPE_ERROR��LOG_TYPE_WARN��LOG_TYPE_INFO��LOG_TYPE_DEBUG
/// @param log_format, ... - ����ַ�����ʽ�������ʹ�÷����� printf һ��
void log_print(int level, char const *log_format, ...);

/// @brief log_set_level ���������־�ļ���
/// @param level - ��־�����ǩ, 4: ERROR, WARNNING, INFO, DEBUG
///                             3: ERROR, WARNNING, INFO
///                             2: ERROR, WARNNING
///                             1: ERROR
///                             0: None
void log_set_level(int level);

#endif // SDK_ARM_LOG_