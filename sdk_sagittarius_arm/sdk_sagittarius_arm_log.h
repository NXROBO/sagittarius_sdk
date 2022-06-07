/*
 * @Descripttion: 日志
 * @version: 1.00
 * @Author: ShuDong.Hong@nxrobo.com
 * @Company: NXROBO (深圳创想未来机器人有限公司)
 * @Date: 2022-03-29 11:10:52
 * @LastEditors: ShuDong.Hong@nxrobo.com
 * @LastEditTime: 2022-03-30 11:49:33
 */
#ifndef SDK_ARM_LOG_
#define SDK_ARM_LOG_
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_constants.h>


// 日志信息的级别
#define LOG_TYPE_DEBUG 4 // 调试信息, 用于调试
#define LOG_TYPE_INFO 3  // 常规信息
#define LOG_TYPE_WARN 2  // 一般警告, 可继续执行但无法保证能正常工作
#define LOG_TYPE_ERROR 1 // 严重错误, 无法继续执行

/// @brief log_print 输出指定级别的日志，第一个参数时日志级别标签，后面参数使用与 printf 一致
/// @param level - 日志级别标签，分别是 LOG_TYPE_ERROR、LOG_TYPE_WARN、LOG_TYPE_INFO、LOG_TYPE_DEBUG
/// @param log_format, ... - 输出字符串格式与参数，使用方法与 printf 一致
void log_print(int level, char const *log_format, ...);

/// @brief log_set_level 设置输出日志的级别
/// @param level - 日志级别标签, 4: ERROR, WARNNING, INFO, DEBUG
///                             3: ERROR, WARNNING, INFO
///                             2: ERROR, WARNNING
///                             1: ERROR
///                             0: None
void log_set_level(int level);

#endif // SDK_ARM_LOG_