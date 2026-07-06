/*
 *	Reverse Proxy Client - Port Proxy Only (Struct Protocol)
 *
 *	- Only port proxy functionality retained
 *	- Device name fixed to "Luckfox", SN from command line
 *	- Exit timer removed, auto-reconnect on control connection loss
 *	- 7 stream types: video0, video1, audio, control, videoCtrl, ssh, rearCam
 *	- Server allocates ports, client receives them via LoginAck
 */

#include "rproxyc.h"

struct rproxyClient rproxy;

unsigned int __host_to_ip(const char *host, unsigned int *ips, int size)
{
	int i=0;
	struct addrinfo hint;
	struct addrinfo *res=NULL,*ai=NULL;

	memset(&hint,0,sizeof(hint));
	hint.ai_family = AF_INET;
	hint.ai_flags = AI_ADDRCONFIG;
	hint.ai_socktype = SOCK_DGRAM;

	if(getaddrinfo(host, NULL, &hint, &res)){
		return 0;
	}

	ai=res;
	while(ai){
		if(ai->ai_family==AF_INET){
			ips[i++]=((struct sockaddr_in *)ai->ai_addr)->sin_addr.s_addr;
			if(i>=size){
				break;
			}
		}
		ai=ai->ai_next;
	}
	if(res){
		freeaddrinfo(res);
	}
	return 0;
}


unsigned int StrToIp(char * stringin)
{
	char * cp;
	int dots = 0;
	int number;
	union{
		unsigned char c[4];
		unsigned int l;
	} retval;
	if(!stringin)
		return 0;

	cp = stringin;
	while(*cp)
	{
		if(*cp > '9' || *cp < '.' || *cp == '/')
			return 0;
		if(*cp == '.')	dots++;
		cp++;
	}

	if( dots != 3 )
		return 0;

	cp = stringin;
	if((number = atoi(cp)) > 255)
		return 0;
	if(number==0)
		return 0;

	retval.c[0] = (unsigned char)number;

	while(*cp != '.')cp++;
	cp++;

	number = atoi(cp);
	while(*cp != '.')cp++;
	cp++;
	if(number > 255) return 0;
	retval.c[1] = (unsigned char)number;


	number = atoi(cp);
	while(*cp != '.')cp++;
	cp++;
	if(number > 255) return 0;
	retval.c[2] = (unsigned char)number;

	if((number = atoi(cp)) >255)
		return 0;
	retval.c[3] = (unsigned char)number;

	return (retval.l);
}

/*域名或IP字符串转成网络序的IP,失败返回0*/
unsigned int hostToIp(const char *host)
{
	unsigned int ip=0;

	ip=StrToIp((char *)host);
	if(ip){
		return ip;
	}

	__host_to_ip(host, &ip, 1);

	return ip;
}


static inline char *ipstr(__u32 ip)
{
	static char str[20]={0};
	unsigned char *ip_dot=(unsigned char *)&ip;

	sprintf(str,"%d.%d.%d.%d",ip_dot[0],ip_dot[1],ip_dot[2],ip_dot[3]);
	return str;
}


/*
	初始化(不再需要portAck)
*/
int rproxyInit(void)
{
	int i;

	for(i=0;i<CONNECT_HLIST_SIZE;i++){
		INIT_HLIST_HEAD(&rproxy.LocalSockHashHead[i]);
	}
	for(i=0;i<CONNECT_HLIST_SIZE;i++){
		INIT_HLIST_HEAD(&rproxy.ServerSockHashHead[i]);
	}

	return 0;
}

struct rproxyConnect *rproxyConnectFind(int sock)
{
	struct rproxyConnect *ct;
	struct hlist_node *pos;

	hlist_for_each(pos, &rproxy.LocalSockHashHead[sockHash(sock)]){
		ct=list_entry(pos, struct rproxyConnect, hashToLocalSock);
		if(sock==ct->localSock){
			ct->from=dataFromLocal;
			return ct;
		}
	}
	hlist_for_each(pos, &rproxy.ServerSockHashHead[sockHash(sock)]){
		ct=list_entry(pos, struct rproxyConnect, hashToServerSock);
		if(sock==ct->serverSock){
			ct->from=dataFromServer;
			return ct;
		}
	}

	return NULL;
}

/*
	查找UDP连接
*/
struct rproxyUdpConnect *rproxyUdpConnectFind(int sock)
{
	if(rproxy.udpControlConn){
		if(sock==rproxy.udpControlConn->localSock || sock==rproxy.udpControlConn->serverSock){
			return rproxy.udpControlConn;
		}
	}
	return NULL;
}

/*
	接收本地UDP数据并转发到服务器
*/
int recvUdpLocalData(struct rproxyUdpConnect *uct)
{
	char buf[OTHER_BUF_SIZE];
	struct sockaddr_in localAddr;
	socklen_t addrLen = sizeof(localAddr);
	int ret;

	ret = recvfrom(uct->localSock, buf, sizeof(buf), 0,
				   (struct sockaddr *)&localAddr, &addrLen);
	if(ret < 0){
		if(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN){
			return 0;
		}
		Printf("UDP recvfrom local error\n");
		return -1;
	}

	/* 转发到服务器(rproxys) */
	ret = sendto(uct->serverSock, buf, ret, 0,
				 (struct sockaddr *)&uct->serverAddr, sizeof(uct->serverAddr));
	if(ret < 0){
		if(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN){
			return 0;
		}
		Printf("UDP sendto server error\n");
		return -1;
	}

	/* 更新发送时间 */
	uct->lastSendTime = time(NULL);

	return 0;
}

/*
	接收服务器UDP数据并转发到本地
*/
int recvUdpServerData(struct rproxyUdpConnect *uct)
{
	char buf[OTHER_BUF_SIZE];
	struct sockaddr_in serverAddr;
	socklen_t addrLen = sizeof(serverAddr);
	int ret;

	ret = recvfrom(uct->serverSock, buf, sizeof(buf), 0,
				   (struct sockaddr *)&serverAddr, &addrLen);
	if(ret < 0){
		if(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN){
			return 0;
		}
		Printf("UDP recvfrom server error\n");
		return -1;
	}

	/* 转发到本地(remote_control) */
	ret = sendto(uct->localSock, buf, ret, 0,
				 (struct sockaddr *)&uct->localAddr, sizeof(uct->localAddr));
	if(ret < 0){
		if(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN){
			return 0;
		}
		Printf("UDP sendto local error\n");
		return -1;
	}

	return 0;
}


int rproxyConnecting(__u32 ip, __u16 port)
{
	int sock;
	struct sockaddr_in addr;

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if(sock<0){
		Printf("socket create failed");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_addr.s_addr=ip;
	addr.sin_port=port;
	addr.sin_family=AF_INET;

	Printf("Connect %s:%d...\n",ipstr(ip),ntohs(port));
	if(connect(sock,(struct sockaddr*)&addr,sizeof(addr))){
		Printf("connect failed\n");
		goto out;
	}
	Printf("Connect ok\n");

	if(setNonblock(sock) < 0){
		Printf("setNonblock falied\n");
		goto out;
	}

	return sock;

out:
	if(sock>0){
		close(sock);
	}
	return -1;
}

void rproxyConnectFree(struct rproxyConnect *ct)
{
	if(ct->localSock!=-1){
		close(ct->localSock);
		hlist_del(&ct->hashToLocalSock);
	}

	if(ct->serverSock!=-1){
		close(ct->serverSock);
		hlist_del(&ct->hashToServerSock);
	}

	if(ct->localPollId!=-1){
		pollDelete(ct->localSock);
	}

	if(ct->serverPollId!=-1){
		pollDelete(ct->serverSock);
	}

	memFree(ct->localBuf);
	memFree(ct->serverBuf);
	memFree(ct);
	rproxy.connectCount--;
	Printf("[%s] connectCount:%d\n",portTypeStr(ct->portType),rproxy.connectCount);
}

/*
	清理所有代理连接(控制连接断开时调用)
*/
void rproxyConnectCleanAll(void)
{
	struct rproxyConnect *ct;
	struct hlist_node *pos, *n;
	int i;

	for(i=0;i<CONNECT_HLIST_SIZE;i++){
		hlist_for_each_entry_safe(ct, pos, n, &rproxy.LocalSockHashHead[i], hashToLocalSock){
			if(ct->localSock!=-1){
				close(ct->localSock);
				pollDelete(ct->localSock);
			}
			if(ct->serverSock!=-1){
				close(ct->serverSock);
				pollDelete(ct->serverSock);
			}
			hlist_del(&ct->hashToLocalSock);
			if(ct->serverSock!=-1){
				hlist_del(&ct->hashToServerSock);
			}
			memFree(ct->localBuf);
			memFree(ct->serverBuf);
			memFree(ct);
			rproxy.connectCount--;
		}
	}
	Printf("CleanAll connectCount:%d\n",rproxy.connectCount);
}

/*
	发送数据到本地服务器
*/
int rproxySendToLocal(struct rproxyConnect *ct)
{
	int ret=0;

	if(!ct->serverBufUsed){
		return 0;
	}
	ret=send(ct->localSock,ct->serverBuf,ct->serverBufUsed,0);
	if(!ret){
		Printf("[%s] local connection closed\n",portTypeStr(ct->portType));
		return -1;
	}
	if(ret<0){
		Printf("ret = %d errno=%d(%s)\n",ret,errno,strerror(errno));
		if(!isIgnoreErrno(errno)){
			Printf("[%s] local connection closed\n",portTypeStr(ct->portType));
			return -1;
		}
		pollOutEvent(ct->localPollId,1);
		goto out;
	}

	ct->serverBufUsed-=ret;
	if(ct->serverBufUsed){
		memmove(ct->serverBuf,ct->serverBuf+ret,ct->serverBufUsed);
		pollOutEvent(ct->localPollId,1);
	}else{
		pollOutEvent(ct->localPollId,0);
	}

out:
	// Printf("ret=%d. left %d bytes\n",ret,ct->serverBufUsed);
	return 0;
}


/*
	发送数据到远端服务器
*/
int rproxySendToServer(struct rproxyConnect *ct)
{
	int ret=0;

	if(!ct->localBufUsed){
		return 0;
	}
	ret=send(ct->serverSock,ct->localBuf,ct->localBufUsed,0);
	if(!ret){
		Printf("[%s] server connection closed\n",portTypeStr(ct->portType));
		return -1;
	}
	if(ret<0){
		Printf("ret = %d errno=%d(%s)\n",ret,errno,strerror(errno));
		if(!isIgnoreErrno(errno)){
			Printf("[%s] server connection closed\n",portTypeStr(ct->portType));
			return -1;
		}
		pollOutEvent(ct->serverPollId,1);
		goto out;
	}

	ct->localBufUsed-=ret;
	if(ct->localBufUsed){
		memmove(ct->localBuf,ct->localBuf+ret,ct->localBufUsed);
		pollOutEvent(ct->serverPollId,1);
	}else{
		pollOutEvent(ct->serverPollId,0);
	}

out:
	// Printf("ret=%d. left %d bytes\n",ret,ct->localBufUsed);
	return 0;
}

int recvLocalData(struct rproxyConnect *ct)
{
	int len;

	if(ct->localBufUsed){	/* 有数据没发送完,等发送完了再接收 */
		return 0;
	}

	len=recv(ct->localSock,ct->localBuf+ct->localBufUsed,ct->localBufSize-ct->localBufUsed,0);
	if(!len){
		Printf("[%s] local connection closed. sock=%d localBufUsed=%d\n",portTypeStr(ct->portType),ct->localSock,ct->localBufUsed);
		if(ct->localBufUsed){
			/* 服务器关闭连接，发送剩余数据 */
			close(ct->localSock);
			pollDelete(ct->localSock);
			ct->localSock=-1;
			ct->localPollId=-1;
			ct->ctState=ctStateLocalClose;
			goto send;
		}
		goto out;
	}
	if(len<0){
		Printf("len=%d errno=%d\n",len,errno);
		if(!isIgnoreErrno(errno)){
			goto out;
		}
		goto send;
	}
	// Printf("recv %d bytes:\n",len);
	ct->localBufUsed+=len;
	ct->localBuf[ct->localBufUsed]=0;

send:
	if(rproxySendToServer(ct)){
		goto out;
	}
	return 0;

out:
	rproxyConnectFree(ct);
	return -1;
}

int recvServerData(struct rproxyConnect *ct)
{
	int len;

	if(ct->serverBufUsed==ct->localBufSize){	/* 没有剩余空间,等发送完了再接收 */
		goto send;
	}

	len=recv(ct->serverSock,ct->serverBuf+ct->serverBufUsed,ct->localBufSize-ct->serverBufUsed,0);
	if(!len){
		Printf("[%s] server connection closed. sock=%d serverBufUsed=%d\n",portTypeStr(ct->portType),ct->serverSock,ct->serverBufUsed);
		goto out;
	}
	if(len<0){
		Printf("len=%d errno=%d\n",len,errno);
		if(!isIgnoreErrno(errno)){
			goto out;
		}
		goto send;
	}
	// Printf("recv %d bytes:\n",len);

	ct->serverBufUsed+=len;

send:
	if(rproxySendToLocal(ct)){
		goto out;
	}
	return 0;

out:
	rproxyConnectFree(ct);
	return -1;
}


/*
	创建一条新的反向代理连接
	portType: 决定使用哪个本地端口
*/
int rproxyNewConnect(__u8 portType, __u32 serverIp, __u16 serverPort)
{
	struct rproxyConnect *ct=memMalloc(sizeof(*ct));

	if(!ct){
		return -1;
	}

	rproxy.connectCount++;

	int bufSize = getBufSizeByPortType(portType);
	ct->localBufSize = bufSize;
	ct->localBuf = memMalloc(bufSize);
	ct->serverBuf = memMalloc(bufSize);
	if(!ct->localBuf || !ct->serverBuf){
		Printf("buffer alloc failed for %s (bufSize=%d)\n", portTypeStr(portType), bufSize);
		goto Err;
	}

	ct->portType=portType;
	ct->localIp=rproxy.localIp;
	ct->localPort=rproxy.localPorts[portType];
	ct->serverIp=serverIp;
	ct->serverPort=serverPort;
	ct->localSock=-1;
	ct->serverSock=-1;
	ct->localPollId=-1;
	ct->serverPollId=-1;

	/* 连接到本地服务器 */
	ct->localSock=rproxyConnecting(ct->localIp,ct->localPort);
	if(ct->localSock==-1){
		goto Err;
	}
	hlist_add_head(&ct->hashToLocalSock, &rproxy.LocalSockHashHead[sockHash(ct->localSock)]);
	ct->localPollId=pollAdd(ct->localSock);
	if(ct->localPollId==-1){
		goto Err;
	}

	/* 连接到远端服务器(proxyPort) */
	ct->serverSock=rproxyConnecting(ct->serverIp,ct->serverPort);
	if(ct->serverSock==-1){
		goto Err;
	}
	hlist_add_head(&ct->hashToServerSock, &rproxy.ServerSockHashHead[sockHash(ct->serverSock)]);
	ct->serverPollId=pollAdd(ct->serverSock);
	if(ct->serverPollId==-1){
		goto Err;
	}
	Printf("serverSock:%d ct->serverPollId=%d rproxy.pollUsed=%d\n",ct->serverSock,ct->serverPollId,rproxy.pollUsed);

	/* 发送BindPortAck消息(标识此proxy连接对应的端口) */
	msg_bind_port_ack_t bpa;
	bpa.head.msgType=msgTypeBindPortAck;
	bpa.head.magic=htons(RPROXY_MSG_MAGIC);
	bpa.head.len=htons(sizeof(bpa));
	bpa.portType=portType;
	bpa.port=rproxy.allocatedPorts[portType];
	bpa.head.checksum=0;
	bpa.head.checksum=htons(rproxy_checksum_calc(&bpa, sizeof(bpa)));

	memcpy(ct->localBuf, &bpa, sizeof(bpa));
	ct->localBufUsed=sizeof(bpa);
	return rproxySendToServer(ct);

Err:
	rproxyConnectFree(ct);
	return -1;
}

/*
	创建UDP客户端连接(用于控制端口低延迟)
*/
int rproxyUdpNewConnect(void)
{
	struct rproxyUdpConnect *uct=memMalloc(sizeof(*uct));
	struct sockaddr_in serverAddr;
	struct sockaddr_in localAddr;

	if(!uct){
		return -1;
	}

	uct->localIp=rproxy.localIp;
	uct->localPort=rproxy.localPorts[portTypeControl];
	uct->serverIp=rproxy.serverIp;
	uct->serverPort=htons(rproxy.udpControlPort);
	uct->portType=portTypeControl;
	uct->localSock=-1;
	uct->serverSock=-1;
	uct->lastSendTime=0;  /* 初始化发送时间 */

	/* 创建本地UDP socket(连接到remote_control) */
	uct->localSock=socket(AF_INET, SOCK_DGRAM, 0);
	if(uct->localSock<0){
		Printf("UDP local socket failed\n");
		goto Err;
	}
	/* 连接到remote_control的UDP端口 */
	memset(&localAddr, 0, sizeof(localAddr));
	localAddr.sin_family=AF_INET;
	localAddr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);  /* localhost */
	localAddr.sin_port=rproxy.localPorts[portTypeControl];  /* remote_control的UDP端口 */
	uct->localAddr=localAddr;  /* 保存remote_control的地址 */
	setNonblock(uct->localSock);
	if(pollAdd(uct->localSock)<0){
		Printf("UDP local pollAdd failed\n");
		goto Err;
	}

	/* 创建服务器UDP socket(连接到rproxys) */
	uct->serverSock=socket(AF_INET, SOCK_DGRAM, 0);
	if(uct->serverSock<0){
		Printf("UDP server socket failed\n");
		goto Err;
	}
	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family=AF_INET;
	serverAddr.sin_addr.s_addr=uct->serverIp;
	serverAddr.sin_port=uct->serverPort;
	uct->serverAddr=serverAddr;
	setNonblock(uct->serverSock);
	if(pollAdd(uct->serverSock)<0){
		Printf("UDP server pollAdd failed\n");
		goto Err;
	}

	Printf("UDP control connection created: local=%d, server=%s:%d\n",
		ntohs(uct->localPort), ipstr(uct->serverIp), ntohs(uct->serverPort));

	rproxy.udpControlConn = uct;  /* 设置全局指针 */

	return 0;

Err:
	if(uct->localSock>=0) close(uct->localSock);
	if(uct->serverSock>=0) close(uct->serverSock);
	memFree(uct);
	return -1;
}


int setNonblock(int sock)
{
	return fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0)|O_NONBLOCK);
}

int setBlocking(int sock)
{
    if(fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) & ~O_NONBLOCK) == -1) {
        return -1;
    }
    return 0;
}

void initList()
{
	int i=0;

	for(i=0;i<POLL_MAX_NUM;i++){
		rproxy.pollArray[i].fd=-1;
	}
	rproxy.pollUsed=0;

}

void pollDelete(int sock)
{
	int i=0;
	int pollUsed=0;

	for(i=0;i<rproxy.pollUsed && i<POLL_MAX_NUM;i++){
		if(rproxy.pollArray[i].fd == sock){
			rproxy.pollArray[i].fd=-1;
		}
		if(rproxy.pollArray[i].fd>0){
			pollUsed=i+1;
		}
	}
	rproxy.pollUsed=pollUsed;
}

int pollAdd(int sock)
{
	int i;

	for(i=0;i<POLL_MAX_NUM;i++){
		if(rproxy.pollArray[i].fd == -1){
			rproxy.pollArray[i].fd=sock;
			break;
		}
	}
	if(i==POLL_MAX_NUM){
		Printf("Add poll %d falied\n",sock);
		return -1;
	}
	rproxy.pollArray[i].events = POLLIN;
	if (i >= rproxy.pollUsed){
		rproxy.pollUsed = i+1;
	}

	return i;
}

void setConnectServerTimer(void)
{
	int time=RPROXY_RECONNECT_TIME;
	mod_timer(&rproxy.connTimer, jiffies+time*HZ);
}

void ConnectServerFun(unsigned long data)
{
	/* 先清理所有残留的代理连接 */
	rproxyConnectCleanAll();

	/* 连接服务器 */
	if(connectRproxyServer()){
		goto err;
	}

	/* 登录 */
	if(sendLoginMsg()){
		rproxyServerSocketClose();
		goto err;
	}

	return ;

err:
	setConnectServerTimer();
}

void initTimer()
{
	srand(time(NULL));
	jiffies_init();
	init_timers_cpu();
	setup_timer(&rproxy.connTimer, ConnectServerFun, 0);
}

void pollOutEvent(int id, int opt)
{
	if(opt){
		rproxy.pollArray[id].events |= POLLOUT;
	}else{
		rproxy.pollArray[id].events &= ~POLLOUT;
	}
}

/*
	控制连接断开后发起重连
*/
void rproxyReconnect(void)
{
	rproxyServerSocketClose();
	setConnectServerTimer();
}

int sendMsgToServer(void *msg, int len)
{
	int ret=0;

	// Printf("send len:%d\n",len);
	if(len>MAX_BUF_LEN-rproxy.sendBufUsed){
		Printf("len=%d sendBufUsed=%d discarded!\n",len,rproxy.sendBufUsed);
		return -1;
	}
	if(msg && len){
		memcpy(rproxy.sendBuf+rproxy.sendBufUsed, msg, len);
	}
	rproxy.sendBufUsed+=len;

	ret=send(rproxy.serverSock,rproxy.sendBuf,rproxy.sendBufUsed,0);
	if(!ret){
		Printf("[control] server connection closed\n");
		rproxy.sendBufUsed=0;
		return -1;
	}
	if(ret<0){
		if(!isIgnoreErrno(errno)){
			Printf("[control] server connection closed\n");
			rproxy.sendBufUsed=0;
			return -1;
		}
		ret=0;
	}
	rproxy.sendBufUsed-=ret;
	if(rproxy.sendBufUsed){
		memmove(rproxy.sendBuf,rproxy.sendBuf+ret,rproxy.sendBufUsed);
		rproxy.pollArray[rproxy.serverPollIndex].events |= POLLOUT;
	}else{
		rproxy.pollArray[rproxy.serverPollIndex].events &= ~POLLOUT;
		// Printf("send ok\n");
	}

	return 0;
}

void rproxyPollDelete(int sock)
{
	int i=0;
	int pollUsed=0;

	for(i=0;i<rproxy.pollUsed && i<POLL_MAX_NUM;i++){
		if(rproxy.pollArray[i].fd == sock){
			rproxy.pollArray[i].fd=-1;
		}
		if(rproxy.pollArray[i].fd>0){
			pollUsed=i+1;
		}
	}
	rproxy.pollUsed=pollUsed;
}


void rproxyServerSocketClose()
{
	if(rproxy.serverSock!=-1){
		close(rproxy.serverSock);
		rproxy.serverPollIndex=-1;
		rproxyPollDelete(rproxy.serverSock);
		rproxy.serverSock=-1;
	}
	rproxy.recvBufUsed=0;
	rproxy.sendBufUsed=0;
	rproxy.timeoutCount=0;
	
	/* 删除UDP心跳定时器 */
	del_timer(&rproxy.udpHeartbeatTimer);
	
	/* 清理UDP连接 */
	if(rproxy.udpControlConn){
		if(rproxy.udpControlConn->localSock>=0){
			rproxyPollDelete(rproxy.udpControlConn->localSock);
			close(rproxy.udpControlConn->localSock);
		}
		if(rproxy.udpControlConn->serverSock>=0){
			rproxyPollDelete(rproxy.udpControlConn->serverSock);
			close(rproxy.udpControlConn->serverSock);
		}
		memFree(rproxy.udpControlConn);
		rproxy.udpControlConn = NULL;
		Printf("UDP connection cleaned up\n");
	}
}

void rproxyKeepAlive(unsigned long data)
{
	msg_keepalive_t msg;

	msg.head.msgType=msgTypeKeepAlive;
	msg.head.magic=htons(RPROXY_MSG_MAGIC);
	msg.head.len=htons(sizeof(msg));
	msg.head.checksum=0;
	msg.head.checksum=htons(rproxy_checksum_calc(&msg, sizeof(msg)));

	if(sendMsgToServer(&msg, sizeof(msg))){
		rproxyServerSocketClose();
		return rproxyReconnect();
	}

	rproxy.timeoutCount++;
	if(rproxy.timeoutCount>=RPROXY_TIMEOUT_COUNT_MAX){
		Printf("Server Close by timeout\n");
		rproxyServerSocketClose();
		return rproxyReconnect();
	}
	Printf("Send KeepAlive.\n");
	mod_timer(&rproxy.timer, jiffies+rproxy.keepAliveTime*HZ);
}

/*
	发送UDP心跳(保持NAT映射表项活跃)
*/
void sendUdpHeartbeat(unsigned long data)
{
	__u32 now = time(NULL);
	char heartbeat[] = "UDP_HB";  /* 简单的心跳数据 */

	if(rproxy.udpControlConn && rproxy.udpControlConn->serverSock>=0){
		/* 如果超过10秒没有发送数据，发送心跳 */
		if(now - rproxy.udpControlConn->lastSendTime >= 10){
			sendto(rproxy.udpControlConn->serverSock, heartbeat, strlen(heartbeat), 0,
				   (struct sockaddr *)&rproxy.udpControlConn->serverAddr, sizeof(rproxy.udpControlConn->serverAddr));
			rproxy.udpControlConn->lastSendTime = now;
			// Printf("UDP heartbeat sent\n");
		}
	}

	/* 重新启动定时器 */
	mod_timer(&rproxy.udpHeartbeatTimer, jiffies + 5*HZ);
}

/*
	处理登录响应: 解析5个分配的端口和心跳时间
*/
int doLoginAck(void *data, int len)
{
	msg_login_ack_t *ack=(msg_login_ack_t *)data;
	int i;

	if(len<(int)sizeof(msg_login_ack_t)){
		Printf("LoginAck too short: %d\n",len);
		return -1;
	}

	/* 保存服务器分配的7个端口(主机序) */
	for(i=0;i<BIND_PORT_NUM;i++){
		rproxy.allocatedPorts[i]=ack->ports[i];
		Printf("%s port: %d\n",portTypeStr(i),ack->ports[i]);
	}

	/* UDP控制端口与控制端口相同 */
	rproxy.udpControlPort = ack->ports[portTypeControl];
	Printf("UDP control port: %d\n", rproxy.udpControlPort);

	/* 保存代理连接端口(转为网络序, 与serverPort保持一致) */
	rproxy.proxyPort=htons(ack->proxyPort);
	Printf("proxyPort: %d\n",ack->proxyPort);

	/* 取心跳时间 */
	rproxy.keepAliveTime=ntohl(ack->keepAliveTime);
	Printf("keepAliveTime=%u\n",rproxy.keepAliveTime);
	if(!rproxy.keepAliveTime){
		rproxy.keepAliveTime=RPROXY_KEEP_ALIVE_TIME;
	}

	del_timer(&rproxy.timer);
	setup_timer(&rproxy.timer, rproxyKeepAlive, 0);
	mod_timer(&rproxy.timer, jiffies+rproxy.keepAliveTime*HZ);

	/* 创建UDP控制连接 */
	if(rproxyUdpNewConnect()==0){
		/* 启动UDP心跳定时器 */
		setup_timer(&rproxy.udpHeartbeatTimer, sendUdpHeartbeat, 0);
		mod_timer(&rproxy.udpHeartbeatTimer, jiffies + 5*HZ);
		Printf("UDP heartbeat timer started\n");
	}else{
		Printf("Failed to create UDP control connection\n");
	}

	return 0;
}


/*
	处理绑定端口通知(服务器通知有新的外部连接)
*/
int doProxy(void *data, int len)
{
	msg_bind_port_t *bp=(msg_bind_port_t *)data;

	if(len<(int)sizeof(msg_bind_port_t)){
		Printf("BindPort too short\n");
		return -1;
	}

	if(bp->portType>=BIND_PORT_NUM){
		Printf("Invalid portType: %d\n",bp->portType);
		return -1;
	}

	Printf("New %s connection, port %d\n",portTypeStr(bp->portType),bp->port);

	/* 根据portType创建代理连接 */
	if(rproxyNewConnect(bp->portType, rproxy.serverIp, rproxy.proxyPort)){
		return -1;
	}

	return 0;
}


/*
	接收服务器消息(无加密, 结构体协议)
*/
int recvProxyServerMsg()
{
	int ret=0;
	int msg_len=0;
	rproxy_msg_head *msg=NULL;

	ret=recv(rproxy.serverSock,rproxy.recvBuf+rproxy.recvBufUsed,MAX_BUF_LEN-rproxy.recvBufUsed,0);
	if(!ret){
		Printf("[control] server connection closed\n");
		return -1;
	}

	if(ret<0){
		if(errno==EINTR || errno==EWOULDBLOCK || errno==EAGAIN){
			ret=0;
		}else{
			Printf("[control] server connection closed\n");
			return -1;
		}
	}
	// Printf("recv %d bytes\n",ret);
	rproxy.recvBufUsed+=ret;

again:
	/* 判断数据长度 */
	if(rproxy.recvBufUsed<(int)sizeof(*msg)){
		Printf("recvBufUsed=%d sizeof(*msg)=%d\n",rproxy.recvBufUsed,(int)sizeof(*msg));
		goto next;
	}

	msg=(rproxy_msg_head *)rproxy.recvBuf;

	/* 检查数据标识 */
	if(ntohs(msg->magic)!=RPROXY_MSG_MAGIC){
		Printf("head->magic:%04X error\n",ntohs(msg->magic));
		goto err;
	}

	/* 判断数据是否接收完 */
	msg_len=ntohs(msg->len);
	if(rproxy.recvBufUsed<msg_len){
		Printf("recvBufUsed=%d msg_len=%d\n",rproxy.recvBufUsed,msg_len);
		goto next;
	}

	if(rproxy_checksum_check(rproxy.recvBuf, msg_len)){
		Printf("checksum error, msgType:%d\n",msg->msgType);
		goto err;
	}

	Printf("msgType:%s\n",msgTypeStr(msg->msgType));
	switch(msg->msgType){
		case msgTypeLoginAck:
			if(doLoginAck(rproxy.recvBuf, msg_len)){
				goto err;
			}
			break;
		case msgTypeKeepAliveAck:
			rproxy.timeoutCount=0;
			break;
		case msgTypeBindPort:
			doProxy(rproxy.recvBuf, msg_len);
			break;
		default:
			Printf("msgType:%d undefined\n",msg->msgType);
			break;
	}

	rproxy.recvBufUsed-=msg_len;
	if(rproxy.recvBufUsed){
		memmove(rproxy.recvBuf,rproxy.recvBuf+msg_len,rproxy.recvBufUsed);
		goto again;
	}

next:
	return 0;

err:
	rproxy.recvBufUsed=0;
	return -1;
}


int setSockTimeout(int sockfd, int sec, int usec)
{
	struct timeval tv;

	tv.tv_sec = sec;
	tv.tv_usec = usec;

	setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	return 0;
}


/*
	连接服务器
*/
int connectRproxyServer()
{
	int sock;
	struct sockaddr_in addr;

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if(sock<0){
		Printf("socket create failed");
		return -1;
	}

	/* 每次连接服务器时重新解析 */
	rproxy.serverIp=hostToIp(rproxy.serverHost);
	memset(&addr, 0, sizeof(addr));
	addr.sin_addr.s_addr=rproxy.serverIp;
	addr.sin_port=rproxy.serverPort;
	addr.sin_family=AF_INET;

	setSockTimeout(sock, 3, 0);

	Printf("Connect %s:%d...\n",ipstr(rproxy.serverIp),ntohs(rproxy.serverPort));
	if(connect(sock,(struct sockaddr*)&addr,sizeof(addr))){
		Printf("connect failed\n");
		goto out;
	}
	Printf("Connect ok\n");

	if(setNonblock(sock) < 0){
		Printf("setNonblock falied\n");
		goto out;
	}
	rproxy.serverPollIndex=pollAdd(sock);
	if(rproxy.serverPollIndex==-1){
		Printf("pollAdd failed\n");
		goto out;
	}
	rproxy.serverSock=sock;

	return 0;

out:
	if(sock>0){
		close(sock);
	}
	return -1;
}

void signalHandler(int sig)
{
	Printf("recv signal:%d\n",sig);
	rproxyServerSocketClose();

	exit(1);
}

int initSignalHandler()
{
	struct sigaction sa;

	memset(&sa,0,sizeof(sa));
	sa.sa_handler = SIG_IGN;
	sigaction(SIGPIPE, &sa, 0);

	memset(&sa,0,sizeof(sa));
	sa.sa_handler = SIG_IGN;
	sigaction(SIGCHLD, &sa, 0);

	signal(SIGHUP, SIG_IGN);

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);
	signal(SIGSEGV, signalHandler);
	signal(SIGBUS, signalHandler);
	signal(SIGABRT, signalHandler);

	return 0;
}


void rproxyRunPoll(int timeout)
{
	int i=0;
	int nready=0;

	nready=poll(rproxy.pollArray, rproxy.pollUsed, timeout);
	if(nready < 1){
		return ;
	}

	for(i=0;i<rproxy.pollUsed;i++){
		if(rproxy.pollArray[i].fd<=0){
			continue;
		}
		if(rproxy.pollArray[i].revents & (POLLIN | POLLOUT | POLLERR)){
			if(rproxy.pollArray[i].fd==rproxy.serverSock){
				if(rproxy.pollArray[i].revents & POLLIN){
					if(recvProxyServerMsg()){
						rproxyServerSocketClose();
						return rproxyReconnect();
					}
				}else{
					rproxyServerSocketClose();
					return rproxyReconnect();
				}
			}else{
				/* 检查是否是UDP连接 */
				struct rproxyUdpConnect *uct=rproxyUdpConnectFind(rproxy.pollArray[i].fd);
				if(uct){
					/* UDP连接 */
					if(rproxy.pollArray[i].revents & POLLIN){
						if(uct->localSock==rproxy.pollArray[i].fd){
							recvUdpLocalData(uct);
						}else if(uct->serverSock==rproxy.pollArray[i].fd){
							recvUdpServerData(uct);
						}
					}
				}else{
					/* TCP连接 */
					struct rproxyConnect *ct=rproxyConnectFind(rproxy.pollArray[i].fd);
					if(!ct){
						Printf("not find ct, sock=%d\n",rproxy.pollArray[i].fd);
						close(rproxy.pollArray[i].fd);
						rproxy.pollArray[i].fd = -1;
					}else if(rproxy.pollArray[i].revents & POLLIN){
						if(ct->from==dataFromLocal){
							recvLocalData(ct);
						}else{
							recvServerData(ct);
						}
					}else if(rproxy.pollArray[i].revents & POLLOUT){
						if(ct->from==dataFromLocal){
							if(rproxySendToLocal(ct)){
								rproxyConnectFree(ct);
							}
						}else{
							if(rproxySendToServer(ct)){
								rproxyConnectFree(ct);
							}
						}
					}else{
						Printf("client.pollArr[%d].revents error\n",i);
						rproxyConnectFree(ct);
					}
				}
			}
			if(--nready <= 0){
				return ;
			}
		}
	}
}

int initRproxyConfig()
{
	rproxy.serverSock=-1;
	rproxy.serverPollIndex=-1;

	return 0;
}

/*
	构建并发送登录消息(结构体协议, 无加密)
*/
int sendLoginMsg(void)
{
	msg_login_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.head.msgType=msgTypeLogin;
	msg.head.magic=htons(RPROXY_MSG_MAGIC);
	msg.head.len=htons(sizeof(msg));
	strncpy(msg.sn, rproxy.sn, SN_MAX_LEN);
	msg.head.checksum=htons(rproxy_checksum_calc(&msg, sizeof(msg)));

	Printf("Login...\n");
	return sendMsgToServer(&msg, sizeof(msg));
}


void help(int argc, char **argv)
{
	printf("Usage:\n");
	printf("\t%s SN serverHost serverPort [debug]\n",argv[0]);
	printf("\n");
	printf("\tSN           - device serial number\n");
	printf("\tserverHost   - proxy server address\n");
	printf("\tserverPort   - proxy server port\n");
	printf("\tdebug        - enable debug output (0 or 1)\n");
}

int main(int argc, char **argv)
{
	initTimer();
	initList();
	initSignalHandler();

	/* 解析命令行参数: SN serverHost serverPort [debug] */
	if(argc<4){
		help(argc,argv);
		return -1;
	}
	strncpy(rproxy.sn,argv[1],sizeof(rproxy.sn)-1);
	rproxy.sn[sizeof(rproxy.sn)-1]='\0';
	strncpy(rproxy.serverHost,argv[2],sizeof(rproxy.serverHost)-1);
	rproxy.serverHost[sizeof(rproxy.serverHost)-1]='\0';
	rproxy.serverPort=htons(atoi(argv[3]));

	/* 硬编码本地IP和端口 */
	rproxy.localIp=StrToIp("127.0.0.1");
	rproxy.localPorts[0]=htons(5100);
	rproxy.localPorts[1]=htons(5101);
	rproxy.localPorts[2]=htons(5102);
	rproxy.localPorts[3]=htons(5103);
	rproxy.localPorts[4]=htons(5104);
	rproxy.localPorts[5]=htons(22);
	rproxy.localPorts[6]=htons(5105);

	if(!rproxy.serverPort){
		help(argc,argv);
		return -1;
	}
	if(argc>=5){
		rproxy.debug = atoi(argv[4]);
	}

	logInit(rproxy.debug);
	atexit(logDeinit);
	Printf("Starting rproxyc\n");

	/* 解析服务器地址 */
	Printf("hostToIp:%s\n",rproxy.serverHost);
	rproxy.serverIp=hostToIp(rproxy.serverHost);
	Printf("serverIp:%s\n",ipstr(rproxy.serverIp));

	if(rproxy.serverIp==0 || rproxy.serverPort==0){
		Printf("Server addr error\n");
	}

	/* 初始化配置 */
	if(initRproxyConfig()){
		Printf("initRproxyConfig failed\n");
		return -1;
	}

	if(rproxyInit()){
		Printf("rproxyInit failed\n");
		return -1;
	}

	/* 连接服务器 */
	if(connectRproxyServer()){
		Printf("connectRproxyServer failed, will retry\n");
		setConnectServerTimer();
		goto loop;
	}

	/* 登录 */
	if(sendLoginMsg()){
		Printf("sendLoginMsg failed, will retry\n");
		rproxyServerSocketClose();
		setConnectServerTimer();
		goto loop;
	}

loop:
	while(1){
		rproxyRunPoll(100);
		run_timers();
	}

	return 0;
}
