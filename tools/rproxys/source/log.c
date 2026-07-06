/*
 * log.c - 日志模块
 *
 * - debug=1: 日志输出到 stdout (调试模式)
 * - debug=0: 日志写入文件，文件保持打开，行缓冲
 * - 写入后检查文件大小，超过阈值时截断保留末尾数据
 * - 截断策略: 文件 > 200KB 时，保留最后 100KB
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdarg.h>
#include "log.h"

#ifdef LOG_THREAD_SAFE
#include <pthread.h>
#endif

static FILE *g_log_fp = NULL;
#ifdef LOG_THREAD_SAFE
static pthread_mutex_t g_log_lock = PTHREAD_MUTEX_INITIALIZER;
#endif

/* 获取当前时间字符串，写入调用者提供的缓冲区 */
static void get_time_str(char *buf, size_t len)
{
	struct timeval tv;
	struct tm *tm;

	gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);
	strftime(buf, len, "%Y-%m-%d %H:%M:%S", tm);
}

/*
 * 检查日志文件大小，超过阈值时截断保留末尾 LOG_MAX_SIZE 字节。
 * 文件保持打开状态，截断后通过 fseek 重置文件位置。
 */
static void logCheckSize(void)
{
	struct stat st;
	int fd, keepSize;
	char *buf;

	/* 先刷出缓冲区，确保 stat 获取准确大小 */
	fflush(g_log_fp);

	if (fstat(fileno(g_log_fp), &st) < 0)
		return;
	if (st.st_size <= LOG_ROTATE_THRESHOLD)
		return;

	/* 保留最后 LOG_MAX_SIZE 字节 */
	keepSize = LOG_MAX_SIZE;
	if (keepSize > st.st_size)
		keepSize = st.st_size;

	buf = malloc(keepSize);
	if (!buf)
		return;

	fd = fileno(g_log_fp);

	/* 读取末尾 keepSize 字节 */
	lseek(fd, -keepSize, SEEK_END);
	if (read(fd, buf, keepSize) != keepSize) {
		free(buf);
		goto reset;
	}

	/* 截断文件并重写 */
	ftruncate(fd, 0);
	lseek(fd, 0, SEEK_SET);
	write(fd, buf, keepSize);
	free(buf);

reset:
	/* 重置 FILE* 位置，后续写入追加到末尾 */
	fseek(g_log_fp, 0, SEEK_END);
}

/*
 * 初始化日志
 *
 * debug=1: 输出到 stdout，不写文件
 * debug=0: 打开日志文件，行缓冲模式
 */
void logInit(int debug)
{
	if (debug) {
		g_log_fp = stdout;
		return;
	}

	/* /var/log 通常已存在，此处仅做防御性检查 */
	if (access("/var/log", F_OK) != 0) {
		mkdir("/var/log", 0755);
	}

	g_log_fp = fopen(LOG_PATH, "a");
	if (!g_log_fp) {
		fprintf(stderr, "logInit: open %s failed: %s\n",
			LOG_PATH, strerror(errno));
		return;
	}

	/* 行缓冲: 每条日志(含\n)立即落盘 */
	setvbuf(g_log_fp, NULL, _IOLBF, 0);
}

/*
 * 关闭日志文件
 */
void logDeinit(void)
{
	if (g_log_fp && g_log_fp != stdout) {
		fflush(g_log_fp);
		fclose(g_log_fp);
	}
	g_log_fp = NULL;
}

/*
 * 写入一条日志记录 (线程安全)
 *
 * 输出格式: "2024-01-15 14:30:25 [funcName:42]:message\n"
 *
 * 文件保持打开，行缓冲自动刷新。
 * 写入后检查文件大小，超限则截断。
 * 整个写入+截断过程在互斥锁内完成，保证多线程安全。
 */
void logPrintf(const char *func, int line, const char *fmt, ...)
{
	char msg[1024];
	char ts[32];
	va_list ap;

	if (!g_log_fp)
		return;

	/* 时间戳写入局部缓冲区，消除静态变量 */
	get_time_str(ts, sizeof(ts));

	/* 格式化消息到局部缓冲区 */
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);

#ifdef LOG_THREAD_SAFE
	pthread_mutex_lock(&g_log_lock);
#endif

	fprintf(g_log_fp, "%s [%s:%d]:%s", ts, func, line, msg);

	/* stdout 模式不检查文件大小 */
	if (g_log_fp != stdout)
		logCheckSize();

#ifdef LOG_THREAD_SAFE
	pthread_mutex_unlock(&g_log_lock);
#endif
}
