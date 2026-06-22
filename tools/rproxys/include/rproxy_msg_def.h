#ifndef __RPROXY_MSG_DEF_H__
#define __RPROXY_MSG_DEF_H__

#define RPROXY_MSG_MAGIC 0xC5D7
#define SN_MAX_LEN 32
#define BIND_PORT_NUM 7    /* 每个设备分配7个端口 */

/* 端口类型 */
enum portType{
	portTypeVideo0 = 0,   /* 高清视频流 */
	portTypeVideo1 = 1,   /* 流畅视频流 */
	portTypeAudio  = 2,   /* 音频流 */
	portTypeControl= 3,   /* 控制流 */
	portTypeVideoCtrl=4,   /* 视频曝光控制 */
	portTypeSSH    = 5,   /* SSH端口22 */
	portTypeRearCam=6,   /* 后置摄像头端口5105 */
};

enum msgType{
	msgTypeNone,
	msgTypeLogin,          /* 登录 */
	msgTypeLoginAck,       /* 登录响应 */
	msgTypeKeepAlive,      /* 保活心跳 */
	msgTypeKeepAliveAck,   /* 保活心跳响应 */
	msgTypeBindPort,       /* 服务器→设备: 通知新外部连接 */
	msgTypeBindPortAck,    /* 设备→服务器: proxy连接标识 */

	msgTypeMax
};

/*
	通信消息头 (packed, 无加密)
*/
typedef struct __attribute__((packed)){
	__u8  msgType;     /* 消息类型 enum msgType */
	__u16 magic;       /* 数据标识 RPROXY_MSG_MAGIC (网络序) */
	__u16 len;         /* 包含头部的总长度 (网络序) */
	__u16 checksum;    /* CRC16校验(网络序), 覆盖整个消息, 校验时此字段视为0 */
} rproxy_msg_head;

/* CRC16-CCITT 查找表 (多项式 0x1021) */
static const __u16 crc16_tab[256]={
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
	0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
	0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
	0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
	0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
	0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
	0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
	0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
	0x4864,0x5845,0x6826,0x7807,0x08E0,0x18C1,0x28A2,0x3883,
	0xC96C,0xD94D,0xE92E,0xF90F,0x89E8,0x99C9,0xA9AA,0xB98B,
	0x5A55,0x4A74,0x7A17,0x6A36,0x1AD1,0x0AF0,0x3A93,0x2AB2,
	0xDB5D,0xCB7C,0xFB1F,0xEB3E,0x9BD9,0x8BF8,0xBB9B,0xAB9A,
	0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
	0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
	0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
	0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
	0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
	0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
	0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
	0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
	0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
	0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
	0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
	0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
	0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
	0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
	0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
	0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
	0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
	0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
	0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
};

/*
	计算CRC16-CCITT校验: 初始值0xFFFF, 多项式0x1021
	调用前消息的checksum字段必须已经为0
*/
static inline __u16 rproxy_checksum_calc(const void *msg, int len)
{
	const unsigned char *p=(const unsigned char *)msg;
	__u16 crc=0xFFFF;
	int i;
	for(i=0;i<len;i++)
		crc=(crc<<8) ^ crc16_tab[((crc>>8) ^ p[i]) & 0xFF];
	return crc;
}

/*
	校验消息: 保存checksum→置0→CRC16重算→比较→还原
	返回: 0=校验通过, -1=校验失败
*/
static inline int rproxy_checksum_check(void *msg, int len)
{
	rproxy_msg_head *h=(rproxy_msg_head *)msg;
	__u16 saved=h->checksum;
	__u16 calc;
	h->checksum=0;
	calc=rproxy_checksum_calc(msg, len);
	h->checksum=saved;
	return (calc==ntohs(saved)) ? 0 : -1;
}

/*
	Login: 设备→服务器
*/
typedef struct __attribute__((packed)){
	rproxy_msg_head head;
	char sn[SN_MAX_LEN];
} msg_login_t;

/*
	LoginAck: 服务器→设备 (分配7个端口 + 代理连接端口)
*/
typedef struct __attribute__((packed)){
	rproxy_msg_head head;
	__u16 ports[BIND_PORT_NUM];   /* 分配的端口号(主机序): video0,video1,audio,control,videoCtrl,ssh,rearCam */
	__u32 keepAliveTime;          /* 心跳间隔(秒), 网络序 */
	__u16 proxyPort;              /* 代理连接端口(主机序) */
} msg_login_ack_t;

/*
	KeepAlive: 设备→服务器
*/
typedef struct __attribute__((packed)){
	rproxy_msg_head head;
} msg_keepalive_t;

/*
	KeepAliveAck: 服务器→设备
*/
typedef struct __attribute__((packed)){
	rproxy_msg_head head;
} msg_keepalive_ack_t;

/*
	BindPort: 服务器→设备(通知有新的外部连接)
*/
typedef struct __attribute__((packed)){
	rproxy_msg_head head;
	__u8  portType;   /* enum portType */
	__u16 port;       /* 主机序端口号 */
} msg_bind_port_t;

/*
	BindPortAck: 设备→服务器(proxy连接标识, 标识此代理连接对应的端口)
*/
typedef struct __attribute__((packed)){
	rproxy_msg_head head;
	__u8  portType;   /* enum portType */
	__u16 port;       /* 主机序端口号 */
} msg_bind_port_ack_t;

static inline char *portTypeStr(int type)
{
	switch(type){
		case portTypeVideo0:  return "video0";
		case portTypeVideo1:  return "video1";
		case portTypeAudio:   return "audio";
		case portTypeControl:   return "control";
		case portTypeVideoCtrl: return "videoCtrl";
		case portTypeSSH:     return "ssh";
		case portTypeRearCam: return "rearCam";
		default:              return "unknown";
	}
}

static inline char *msgTypeStr(int msgType)
{
	switch(msgType){
		case msgTypeNone:         return "None";
		case msgTypeLogin:        return "Login";
		case msgTypeLoginAck:     return "LoginAck";
		case msgTypeKeepAlive:    return "KeepAlive";
		case msgTypeKeepAliveAck: return "KeepAliveAck";
		case msgTypeBindPort:     return "BindPort";
		case msgTypeBindPortAck:  return "BindPortAck";
		default:                  return "Undefined";
	}
}

#endif /* __RPROXY_MSG_DEF_H__ */
