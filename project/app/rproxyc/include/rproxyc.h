
#ifndef __RPROXYC_H__
#define __RPROXYC_H__

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
#include <list.h>
#include <dirent.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <linux/types.h>
#include <linux/netlink.h>
#include <timer.h>

#include "rproxy_msg_def.h"
#include "log.h"

#define MAX_BUF_LEN 10240
#define RPROXY_KEEP_ALIVE_TIME 60          /* 1分钟 */
#define RPROXY_TIMEOUT_COUNT_MAX 2
#define POLL_MAX_NUM 1024
#define RPROXY_RECONNECT_TIME 5            /* 断开后5秒重连 */
/* Printf 宏已移至 log.h */

#define CONNECT_HLIST_MASK 0xFF
#define CONNECT_HLIST_SIZE 0x100
#define VIDEO_BUF_SIZE (128 * 1024)   /* video0/video1/rearCam: 容纳大I帧 */
#define AUDIO_BUF_SIZE (32 * 1024)    /* audio */
#define OTHER_BUF_SIZE (8 * 1024)     /* control/videoCtrl/ssh */

static inline int getBufSizeByPortType(__u8 portType)
{
	switch(portType){
		case portTypeVideo0:
		case portTypeVideo1:
		case portTypeRearCam:
			return VIDEO_BUF_SIZE;
		case portTypeAudio:
			return AUDIO_BUF_SIZE;
		default:
			return OTHER_BUF_SIZE;
	}
}

enum{
	dataFromLocal,
	dataFromServer,
};

enum{
	ctStateInit=0,
	ctStateLocalClose,    /* 本地服务器已关闭了连接 */
};


/* 一条反向代理的连接 */
struct rproxyConnect{
	__u32 localIp;              /* 本地服务器的IP */
	__u16 localPort;            /* 本地服务器的端口(网络序) */
	__u32 serverIp;             /* 远端服务器的IP */
	__u16 serverPort;           /* 远端服务器的端口(网络序) */

	int localSock;              /* 本地服务器的Socket */
	int serverSock;             /* 远端服务器的Socket */

	char *localBuf;               /* 从本地服务器收到的数据,或发往远端服务器的数据 */
	char *serverBuf;              /* 从远端服务器收到的数据,或发往本地服务器的数据 */
	int localBufSize;             /* 缓冲区大小(按portType分配) */
	int localBufUsed;
	int serverBufUsed;

	int from;                   /* 当前应该从本地服务器还是远端服务器接收数据 */
	int localPollId;            /* localSock 在poll中的ID */
	int serverPollId;           /* serverSock 在poll中的ID */
	int ctState;                /* 连接的状态 */

	__u8 portType;              /* 端口类型 enum portType */

	struct hlist_node hashToLocalSock;
	struct hlist_node hashToServerSock;
};

/* UDP连接结构体 */
struct rproxyUdpConnect{
	__u32 localIp;              /* 本地IP */
	__u16 localPort;            /* 本地UDP端口(网络序) */
	__u32 serverIp;             /* 服务器IP */
	__u16 serverPort;           /* 服务器UDP端口(网络序) */
	
	int localSock;              /* 本地UDP socket */
	int serverSock;             /* 服务器UDP socket */
	
	struct sockaddr_in localAddr;   /* remote_control的地址 */
	struct sockaddr_in serverAddr;  /* rproxys的地址 */
	
	__u8 portType;              /* 端口类型 */
	
	__u32 lastSendTime;         /* 上次发送时间(秒) */
	
	struct hlist_node hashToLocalSock;
};

struct rproxyClient{
	struct pollfd pollArray[POLL_MAX_NUM];
	struct timer_list timer;           /* keep alive timer */
	struct timer_list connTimer;       /* Connect timer */
	struct timer_list udpHeartbeatTimer;  /* UDP heartbeat timer */
	int pollUsed;
	int serverSock;                    /* 连接服务器的socket */
	int serverPollIndex;               /* 在pollArray中的位置 */
	char serverHost[64];               /* 服务器地址 */
	__u32 serverIp;                    /* 服务器IP, 网络序 */
	__u16 serverPort;                  /* 服务器端口(devPort), 网络序 */
	__u16 proxyPort;                   /* 服务器代理连接端口(网络序), 从LoginAck获取 */
	int timeoutCount;

	__u32 localIp;                     /* 本地IP */
	__u16 localPorts[BIND_PORT_NUM];   /* 7个本地端口(网络序): video0,video1,audio,control,videoCtrl,ssh,rearCam */
	__u16 allocatedPorts[BIND_PORT_NUM];/* 服务器分配的端口(主机序) */
	__u16 udpControlPort;              /* 服务器分配的UDP控制端口(主机序) */

	char sn[SN_MAX_LEN];               /* 设备SN */
	char sendBuf[MAX_BUF_LEN];
	char recvBuf[MAX_BUF_LEN];
	int sendBufUsed;
	int recvBufUsed;
	int debug;
	__u32 keepAliveTime;
	__u32 connectCount;                /* 当前代理连接数 */

	struct hlist_head LocalSockHashHead[CONNECT_HLIST_SIZE];
	struct hlist_head ServerSockHashHead[CONNECT_HLIST_SIZE];
	struct hlist_head UdpSockHashHead[CONNECT_HLIST_SIZE];
	struct rproxyUdpConnect *udpControlConn;  /* UDP控制连接全局指针 */
};

extern struct rproxyClient rproxy;


static inline int isIgnoreErrno(int errNo)
{
	switch(errNo)
	{
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
	if(p){
		memset(p, 0, size);
	}
	return p;
}
static inline void memFree(void *buf)
{
	if(buf){
		free(buf);
	}
}

static inline unsigned int sockHash(int sock)
{
	return sock&CONNECT_HLIST_MASK;
}

unsigned int hostToIp(const char *host);

int setNonblock(int sock);
void pollOutEvent(int id, int opt);
void pollDelete(int sock);
int pollAdd(int sock);

int rproxyNewConnect(__u8 portType, __u32 serverIp, __u16 serverPort);
struct rproxyConnect *rproxyConnectFind(int sock);
int recvLocalData(struct rproxyConnect *ct);
int recvServerData(struct rproxyConnect *ct);
int rproxySendToLocal(struct rproxyConnect *ct);
int rproxySendToServer(struct rproxyConnect *ct);
void rproxyConnectFree(struct rproxyConnect *ct);
void rproxyConnectCleanAll(void);
int connectRproxyServer(void);
int sendLoginMsg(void);
void rproxyServerSocketClose(void);
void rproxyReconnect(void);
void sendUdpHeartbeat(unsigned long data);

#endif  /* __RPROXY_H__ */
