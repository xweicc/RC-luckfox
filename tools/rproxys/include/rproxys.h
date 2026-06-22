#ifndef __RPROXYS_H__
#define __RPROXYS_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <netdb.h>
#include <ctype.h>
#include <assert.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <linux/types.h>

#include <list.h>
#include <timer.h>

#include "rproxy_msg_def.h"

#define HLIST_MAX  0x10000
#define HLIST_MASK 0xFFFF

#define MAX_BUF_LEN    65536
#define EPOLL_MAX_NUM  1024

/* 超时时间(秒) */
#define DEV_CONN_TIMEOUT             60
#define PROXY_CONN_TIMEOUT           60
#define DEV_CONN_CONFIRM_TIMEOUT     120
#define PROXY_CONN_CONFIRM_TIMEOUT   3600
#define QUERY_CONN_TIMEOUT           10

#define Printf(format,args...) \
	do{ if(rps.debug) printf("[%s:%d]:"format,__FUNCTION__,__LINE__,##args); }while(0)

enum proxyConnState{
	proxyConnInit,
	proxyConnConfirm,
};

struct proxyClientConn;
struct devClientConn;
struct queryClientConn;

/*
	绑定端口的外部连接 (accept后的客户端连接)
*/
typedef struct {
	struct hlist_node hashToPort;
	struct hlist_node hashToSock;
	struct timer_list timer;
	int sock;
	__u16 port;         /* 主机序 */
	__u8  portType;     /* enum portType */
	char recvBuf[MAX_BUF_LEN];
	int recvBufUsed;
	int state;
	struct proxyClientConn *proxyCt;
} bindClientConn;

/*
	绑定监听器 (每个设备7个监听端口)
*/
typedef struct {
	struct hlist_node hashToSock;
	int sock;
	__u16 port;         /* 主机序 */
	__u8  portType;     /* enum portType */
	struct devClientConn *devCt;
} bindListenerConn;

/*
	设备连接
*/
typedef struct devClientConn{
	struct hlist_node hashToSn;
	struct hlist_node hashToSock;
	struct timer_list timer;
	int devSock;
	char sn[SN_MAX_LEN];
	char recvBuf[MAX_BUF_LEN];
	int recvBufUsed;
	char sendBuf[MAX_BUF_LEN];
	int sendBufUsed;

	/* 7个绑定端口 */
	struct {
		bindListenerConn *listener;
		__u16 port;       /* 主机序, 0=未分配 */
	} bind[BIND_PORT_NUM];

	__u32 ip;
	__u32 deviceIp;
	char version[8];
} devClientConn;

/*
	代理连接 (设备侧代理通道)
*/
typedef struct proxyClientConn{
	struct hlist_node hashToSock;
	struct timer_list timer;
	int proxySock;
	char sn[SN_MAX_LEN];
	char recvBuf[MAX_BUF_LEN];
	int recvBufUsed;
	int state;
	__u16 port;         /* 主机序 */
	bindClientConn *bindCt;
} proxyClientConn;

/*
	APP查询端口连接 (短连接, 非阻塞)
*/
#define QUERY_BUF_LEN 256

typedef struct queryClientConn{
	struct hlist_node hashToSock;
	struct timer_list timer;
	int sock;
	char recvBuf[QUERY_BUF_LEN];
	int recvBufUsed;
	char sendBuf[QUERY_BUF_LEN];
	int sendBufUsed;
} queryClientConn;

/*
	端口池管理
*/
struct portPool{
	int rangeStart;     /* 起始端口 */
	int rangeEnd;       /* 结束端口(不含) */
	char *used;         /* 位图: used[port - rangeStart] */
	int totalPorts;     /* rangeEnd - rangeStart */
};

/*
	反向代理服务器全局状态
*/
struct reverseProxyServer{
	int epollFd;
	int epollMaxEvents;
	int debug;

	int devSock;        /* 设备连接监听端口 */
	int proxySock;      /* 代理连接监听端口 */
	int querySock;      /* APP查询端口监听端口 */

	__u16 devPort;      /* 设备连接端口 */
	__u16 proxyPort;    /* 代理连接端口 */
	__u16 queryPort;    /* APP查询端口 */
	char configFile[64];

	struct portPool pool;

	struct hlist_head devSnHlist[HLIST_MAX];
	struct hlist_head bindPortHlist[HLIST_MAX];

	struct hlist_head devSockHlist[HLIST_MAX];
	struct hlist_head proxySockHlist[HLIST_MAX];
	struct hlist_head bindSockHlist[HLIST_MAX];
	struct hlist_head bindListenerSockHlist[HLIST_MAX];
	struct hlist_head querySockHlist[HLIST_MAX];
};

extern struct reverseProxyServer rps;

/* ---- 工具函数 ---- */

static inline int isIgnoreErrno(int errNo)
{
	switch(errNo){
		case 0:
		case EINPROGRESS:
		case EWOULDBLOCK:
		case EALREADY:
		case EINTR:
		case ERESTART:
			return 1;
		default:
			return 0;
	}
}

static inline void *memMalloc(int size)
{
	void *p=malloc(size);
	if(p) memset(p, 0, size);
	return p;
}

static inline void memFree(void *buf)
{
	if(buf) free(buf);
}

static inline char *ipstr(__u32 ip)
{
	static char str[20]={0};
	unsigned char *d=(unsigned char *)&ip;
	sprintf(str,"%d.%d.%d.%d",d[0],d[1],d[2],d[3]);
	return str;
}

static inline char *ipstr2(__u32 ip)
{
	static char str[20]={0};
	unsigned char *d=(unsigned char *)&ip;
	sprintf(str,"%d.%d.%d.%d",d[0],d[1],d[2],d[3]);
	return str;
}

static inline int sockHash(int sock)
{
	return sock & HLIST_MASK;
}

static inline int portHash(__u16 port)
{
	return port & HLIST_MASK;
}

static inline unsigned int SnHash(char *str)
{
	unsigned int hash=0, seed=131;
	int i=0;
	while(*str && i<SN_MAX_LEN){
		hash=hash*seed+(*str++);
		i++;
	}
	return hash & HLIST_MASK;
}

#endif /* __RPROXY_H__ */
