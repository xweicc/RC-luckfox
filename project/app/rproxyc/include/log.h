/*
 * log.h - rproxyc 日志模块
 *
 * debug=1: 日志输出到 stdout
 * debug=0: 日志写入文件，自动截断保留最新 100KB
 */

#ifndef __LOG_H__
#define __LOG_H__

#include <stdio.h>
#include <stdarg.h>

#define LOG_PATH        "/oem/rproxyc.log"
#define LOG_MAX_SIZE    (100 * 1024)   /* 最终保留 100KB */
#define LOG_ROTATE_THRESHOLD  (150 * 1024)  /* 截断触发阈值: 超过此值才截断 */

void logInit(int debug);
void logDeinit(void);
void logPrintf(const char *func, int line, const char *fmt, ...);

#define Printf(fmt, ...) \
	logPrintf(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#endif /* __LOG_H__ */
