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
#include "log.h"

#define HLIST_MAX  0x10000
#define HLIST_MASK 0xFFFF

#define DEV_BUF_LEN    8192        /* devClientConn: 控制通道, 小消息 */
#define PROXY_BUF_LEN  (128 * 1024) /* proxyClientConn: 代理数据通道 */
#define BIND_BUF_LEN   (128 * 1024) /* bindClientConn: 外部客户端连接 */
#define EPOLL_MAX_NUM  1024

/* 超时时间(秒) */
#define DEV_CONN_TIMEOUT             60
#define PROXY_CONN_TIMEOUT           60
#define DEV_CONN_CONFIRM_TIMEOUT     120
#define PROXY_CONN_CONFIRM_TIMEOUT   3600
#define QUERY_CONN_TIMEOUT           10

/* Printf 宏已移至 log.h */

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
	char recvBuf[BIND_BUF_LEN];
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
	UDP 绑定监听器
*/
typedef struct {
	struct hlist_node hashToSock;
	int sock;
	__u16 port;         /* 主机序 */
	__u8  portType;     /* enum portType */
	struct devClientConn *devCt;
	struct sockaddr_in clientAddr;  /* APP的UDP地址 */
	int hasClient;      /* 是否有客户端连接 */
	struct sockaddr_in proxyAddr;  /* rproxyc的UDP地址 */
} udpBindListenerConn;

/*
	设备连接
*/
typedef struct devClientConn{
	struct hlist_node hashToSn;
	struct hlist_node hashToSock;
	struct timer_list timer;
	int devSock;
	char sn[SN_MAX_LEN];
	char recvBuf[DEV_BUF_LEN];
	int recvBufUsed;
	char sendBuf[DEV_BUF_LEN];
	int sendBufUsed;

	/* 7个绑定端口 (TCP) */
	struct {
		bindListenerConn *listener;
		__u16 port;       /* 主机序, 0=未分配 */
	} bind[BIND_PORT_NUM];

	/* UDP控制端口 */
	struct {
		udpBindListenerConn *udpListener;
		__u16 udpPort;    /* 主机序, 0=未分配 */
	} udpControl;

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
	char recvBuf[PROXY_BUF_LEN];
	int recvBufUsed;
	int state;
	__u16 port;         /* 主机序 */
	bindClientConn *bindCt;
	struct sockaddr_in proxyAddr;  /* rproxyc的地址 */
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
	struct hlist_head udpBindListenerSockHlist[HLIST_MAX];  /* UDP监听器哈希表 */
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
