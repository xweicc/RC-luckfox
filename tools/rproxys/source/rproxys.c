/*
 *  Reverse Proxy Server (Port Proxy Only, Struct Protocol)
 *  服务器管理端口池, 登录时分配7个端口给设备
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation.
 */

#include "rproxys.h"

struct reverseProxyServer rps;

int setNonblock(int sock)
{
	return fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0)|O_NONBLOCK);
}

int epollInit()
{
	rps.epollFd = epoll_create(EPOLL_MAX_NUM);
	if(rps.epollFd==-1){
		Printf("epoll_create: %s\n",strerror(errno));
		return -1;
	}
	return 0;
}

void epollDelete(int sock)
{
	if(epoll_ctl(rps.epollFd, EPOLL_CTL_DEL, sock, NULL)==0){
		rps.epollMaxEvents--;
	}else{
		Printf("epollDelete fd=%d failed: %s\n",sock,strerror(errno));
	}
}

int epollAdd(int sock)
{
	int ret;
	struct epoll_event event;

	bzero(&event, sizeof(struct epoll_event));
	event.data.fd=sock;
	event.events=EPOLLIN;
	ret=epoll_ctl(rps.epollFd, EPOLL_CTL_ADD, sock, &event);
	if(ret){
		Printf("epoll_ctl: %s\n",strerror(errno));
		return -1;
	}
	rps.epollMaxEvents++;
	return 0;
}

int epollOutEvent(int sock, int opt)
{
	struct epoll_event event;

	bzero(&event, sizeof(struct epoll_event));
	event.data.fd=sock;
	if(opt){
		event.events = EPOLLIN|EPOLLOUT;
	}else{
		event.events = EPOLLIN;
	}
	return epoll_ctl(rps.epollFd, EPOLL_CTL_MOD, sock, &event);
}

void initTimer()
{
	srand(time(NULL));
	jiffies_init();
	init_timers_cpu();
}

void signalHandler(int sig)
{
	Printf("recv signal:%d\n",sig);
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

void initList()
{
	int i;
	for(i=0;i<HLIST_MAX;i++){
		INIT_HLIST_HEAD(&rps.devSnHlist[i]);
		INIT_HLIST_HEAD(&rps.bindPortHlist[i]);
		INIT_HLIST_HEAD(&rps.devSockHlist[i]);
		INIT_HLIST_HEAD(&rps.proxySockHlist[i]);
		INIT_HLIST_HEAD(&rps.bindSockHlist[i]);
		INIT_HLIST_HEAD(&rps.bindListenerSockHlist[i]);
		INIT_HLIST_HEAD(&rps.querySockHlist[i]);
	}
}

/* ======================== 端口池管理 ======================== */

int initPortPool(int rangeStart, int rangeEnd)
{
	int total=rangeEnd-rangeStart;

	if(total<=0){
		Printf("Invalid port range: %d-%d\n",rangeStart,rangeEnd);
		return -1;
	}

	rps.pool.rangeStart=rangeStart;
	rps.pool.rangeEnd=rangeEnd;
	rps.pool.totalPorts=total;
	rps.pool.used=memMalloc(total);
	if(!rps.pool.used){
		Printf("memMalloc failed\n");
		return -1;
	}

	return 0;
}

void releasePortPool()
{
	if(rps.pool.used){
		memFree(rps.pool.used);
		rps.pool.used=NULL;
	}
}

/* 分配一个端口, 返回主机序端口号, 失败返回0 */
__u16 portAllocate()
{
	int i;
	for(i=0; i<rps.pool.totalPorts; i++){
		if(!rps.pool.used[i]){
			rps.pool.used[i]=1;
			return (__u16)(rps.pool.rangeStart + i);
		}
	}
	return 0;
}

/* 释放一个端口 */
void portRelease(__u16 port)
{
	int idx=port - rps.pool.rangeStart;
	if(idx>=0 && idx<rps.pool.totalPorts){
		rps.pool.used[idx]=0;
	}
}

/* ======================== 查找函数 ======================== */

devClientConn *devClientConnFindBySn(char *sn)
{
	struct hlist_node *pos;
	devClientConn *ct;
	hlist_for_each(pos, &rps.devSnHlist[SnHash(sn)]){
		ct=hlist_entry(pos, devClientConn, hashToSn);
		if(strncmp(ct->sn,sn,sizeof(ct->sn))==0){
			Printf("ct->sn:%s,sn:%s\n",ct->sn,sn);
			return ct;
		}
	}
	return NULL;
}

devClientConn *devClientFind(int sock)
{
	struct hlist_node *pos;
	devClientConn *ct;
	hlist_for_each(pos, &rps.devSockHlist[sockHash(sock)]){
		ct=hlist_entry(pos, devClientConn, hashToSock);
		if(ct->devSock==sock){
			return ct;
		}
	}
	return NULL;
}

proxyClientConn *proxyClientFind(int sock)
{
	struct hlist_node *pos;
	proxyClientConn *ct;
	hlist_for_each(pos, &rps.proxySockHlist[sockHash(sock)]){
		ct=hlist_entry(pos, proxyClientConn, hashToSock);
		if(ct->proxySock==sock){
			mod_timer(&ct->timer, jiffies+PROXY_CONN_CONFIRM_TIMEOUT*HZ);
			return ct;
		}
	}
	return NULL;
}

bindClientConn *bindPortClientFind(int sock)
{
	struct hlist_node *pos;
	bindClientConn *ct;
	hlist_for_each(pos, &rps.bindSockHlist[sockHash(sock)]){
		ct=hlist_entry(pos, bindClientConn, hashToSock);
		if(ct->sock==sock){
			if(ct->state==proxyConnConfirm){
				mod_timer(&ct->timer, jiffies+PROXY_CONN_CONFIRM_TIMEOUT*HZ);
			}else{
				mod_timer(&ct->timer, jiffies+PROXY_CONN_TIMEOUT*HZ);
			}
			return ct;
		}
	}
	return NULL;
}

bindClientConn *bindClientFindPort(__u16 port)
{
	struct hlist_node *pos;
	bindClientConn *ct;
	hlist_for_each(pos, &rps.bindPortHlist[portHash(port)]){
		ct=hlist_entry(pos, bindClientConn, hashToPort);
		if(ct->port==port){
			hlist_del_init(pos);
			return ct;
		}
	}
	return NULL;
}

bindListenerConn *bindListenerFind(int sock)
{
	struct hlist_node *pos;
	bindListenerConn *lt;
	hlist_for_each(pos, &rps.bindListenerSockHlist[sockHash(sock)]){
		lt=hlist_entry(pos, bindListenerConn, hashToSock);
		if(lt->sock==sock){
			return lt;
		}
	}
	return NULL;
}

/* ======================== 发送数据到设备 ======================== */

int sendDataToDevClient(devClientConn *ct, void *msg, int len)
{
	int ret=0;

	if(len>sizeof(ct->sendBuf)-ct->sendBufUsed){
		Printf("len=%d sendBufUsed=%d discarded!\n",len,ct->sendBufUsed);
		return -1;
	}
	if(msg && len){
		memcpy(ct->sendBuf+ct->sendBufUsed, msg, len);
	}
	ct->sendBufUsed+=len;

	ret=send(ct->devSock,ct->sendBuf,ct->sendBufUsed,0);
	if(!ret){
		Printf("Connection closed by peer\n");
		ct->sendBufUsed=0;
		return -1;
	}
	if(ret<0){
		if(!isIgnoreErrno(errno)){
			Printf("Connection closed by peer\n");
			ct->sendBufUsed=0;
			return -1;
		}
		ret=0;
		epollOutEvent(ct->devSock,1);
		goto out;
	}
	ct->sendBufUsed-=ret;
	if(ct->sendBufUsed){
		memmove(ct->sendBuf,ct->sendBuf+ret,ct->sendBufUsed);
		epollOutEvent(ct->devSock,1);
	}else{
		epollOutEvent(ct->devSock,0);
	}

out:
	return 0;
}

/* ======================== 构建消息 ======================== */

int bulidLoginAckMsg(char *buf, int size, __u16 ports[BIND_PORT_NUM], __u16 proxyPort)
{
	msg_login_ack_t *msg=(msg_login_ack_t *)buf;
	int i;

	if((int)sizeof(*msg)>size) return -1;

	msg->head.msgType=msgTypeLoginAck;
	msg->head.magic=htons(RPROXY_MSG_MAGIC);
	msg->head.len=htons(sizeof(*msg));
	for(i=0;i<BIND_PORT_NUM;i++){
		msg->ports[i]=ports[i];  /* 主机序 */
	}
	msg->keepAliveTime=htonl(DEV_CONN_CONFIRM_TIMEOUT/2);
	msg->proxyPort=proxyPort;    /* 主机序, 告诉设备代理连接用哪个端口 */
	msg->head.checksum=0;
	msg->head.checksum=htons(rproxy_checksum_calc(msg, sizeof(*msg)));

	return sizeof(*msg);
}

int bulidKeepAliveAckMsg(char *buf, int size)
{
	msg_keepalive_ack_t *msg=(msg_keepalive_ack_t *)buf;

	if((int)sizeof(*msg)>size) return -1;

	msg->head.msgType=msgTypeKeepAliveAck;
	msg->head.magic=htons(RPROXY_MSG_MAGIC);
	msg->head.len=htons(sizeof(*msg));
	msg->head.checksum=0;
	msg->head.checksum=htons(rproxy_checksum_calc(msg, sizeof(*msg)));

	return sizeof(*msg);
}

/*
	构建BindPort通知消息(服务器→设备, 通知有新的外部连接)
*/
int bulidBindPortMsg(char *buf, int size, __u8 portType, __u16 port)
{
	msg_bind_port_t *msg=(msg_bind_port_t *)buf;

	if((int)sizeof(*msg)>size) return -1;

	msg->head.msgType=msgTypeBindPort;
	msg->head.magic=htons(RPROXY_MSG_MAGIC);
	msg->head.len=htons(sizeof(*msg));
	msg->portType=portType;
	msg->port=port;
	msg->head.checksum=0;
	msg->head.checksum=htons(rproxy_checksum_calc(msg, sizeof(*msg)));

	return sizeof(*msg);
}

/* ======================== 发送消息 ======================== */

int replyDevClientLoginAck(devClientConn *ct, __u16 ports[BIND_PORT_NUM])
{
	char buf[256]={0};
	int len=bulidLoginAckMsg(buf, sizeof(buf), ports, ntohs(rps.proxyPort));
	if(len==-1) return -1;
	return sendDataToDevClient(ct, buf, len);
}

int replyKeepAliveAck(devClientConn *ct)
{
	char buf[256]={0};
	int len=bulidKeepAliveAckMsg(buf, sizeof(buf));
	if(len==-1) return -1;
	return sendDataToDevClient(ct, buf, len);
}

int sendBindPortNotify(devClientConn *devCt, __u8 portType, __u16 port)
{
	char buf[256]={0};
	int len=bulidBindPortMsg(buf, sizeof(buf), portType, port);
	if(len==-1) return -1;
	return sendDataToDevClient(devCt, buf, len);
}

/* ======================== 设备消息处理 ======================== */

/*
	创建设备的绑定监听端口
*/
bindListenerConn *createBindListener(devClientConn *ct, __u8 portType, __u16 port)
{
	int sock;
	struct sockaddr_in my_addr;
	bindListenerConn *lt;
	int optval=1;

	sock=socket(AF_INET,SOCK_STREAM,0);
	if(sock==-1){
		Printf("socket: %s\n",strerror(errno));
		return NULL;
	}

	bzero(&my_addr, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(port);
	my_addr.sin_addr.s_addr = INADDR_ANY;

	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(optval));
	if(bind(sock, (struct sockaddr *)&my_addr, sizeof(struct sockaddr))==-1){
		Printf("bind %d: %s\n",port,strerror(errno));
		goto out;
	}

	if(listen(sock, SOMAXCONN)==-1){
		Printf("listen: %s\n",strerror(errno));
		goto out;
	}
	setNonblock(sock);
	if(epollAdd(sock)){
		goto out;
	}

	lt=memMalloc(sizeof(*lt));
	if(!lt){
		goto out;
	}

	lt->sock=sock;
	lt->port=port;
	lt->portType=portType;
	lt->devCt=ct;
	INIT_HLIST_NODE(&lt->hashToSock);
	hlist_add_head(&lt->hashToSock, &rps.bindListenerSockHlist[sockHash(sock)]);

	return lt;

out:
	close(sock);
	return NULL;
}

/*
	删除绑定监听器
*/
void deleteBindListener(bindListenerConn *lt)
{
	hlist_del(&lt->hashToSock);
	epollDelete(lt->sock);
	close(lt->sock);
	memFree(lt);
}

/*
	为设备分配7个端口并创建监听
*/
int allocatePortsForDevice(devClientConn *ct)
{
	int i;
	__u16 ports[BIND_PORT_NUM];

	for(i=0;i<BIND_PORT_NUM;i++){
		ports[i]=portAllocate();
		if(!ports[i]){
			Printf("No port available for %s type %s\n",ct->sn,portTypeStr(i));
			/* 释放已分配的 */
			int j;
			for(j=0;j<i;j++){
				if(ct->bind[j].listener){
					deleteBindListener(ct->bind[j].listener);
					ct->bind[j].listener=NULL;
				}
				portRelease(ct->bind[j].port);
				ct->bind[j].port=0;
			}
			return -1;
		}

		ct->bind[i].port=ports[i];
		ct->bind[i].listener=createBindListener(ct, (__u8)i, ports[i]);
		if(!ct->bind[i].listener){
			Printf("createBindListener %d failed for %s type %s\n",
				ports[i], ct->sn, portTypeStr(i));
			portRelease(ports[i]);
			ct->bind[i].port=0;
			/* 释放已分配的 */
			int j;
			for(j=0;j<i;j++){
				if(ct->bind[j].listener){
					deleteBindListener(ct->bind[j].listener);
					ct->bind[j].listener=NULL;
				}
				portRelease(ct->bind[j].port);
				ct->bind[j].port=0;
			}
			return -1;
		}

		Printf("%s allocated %s port %d\n",ct->sn,portTypeStr(i),ports[i]);
	}

	/* 发送 LoginAck(包含7个端口) */
	if(replyDevClientLoginAck(ct, ports)){
		return -1;
	}

	return 0;
}

/*
	释放设备的所有端口
*/
void releasePortsForDevice(devClientConn *ct)
{
	int i;
	for(i=0;i<BIND_PORT_NUM;i++){
		if(ct->bind[i].listener){
			deleteBindListener(ct->bind[i].listener);
			ct->bind[i].listener=NULL;
		}
		if(ct->bind[i].port){
			Printf("%s release %s port %d\n",ct->sn,portTypeStr(i),ct->bind[i].port);
			portRelease(ct->bind[i].port);
			ct->bind[i].port=0;
		}
	}
}

int devClientLogin(devClientConn *ct, void *data, int len)
{
	msg_login_t *msg=(msg_login_t *)data;

	if(len<(int)sizeof(msg_login_t)-sizeof(rproxy_msg_head)){
		Printf("login data too short\n");
		return -1;
	}

	if(msg->sn[0]=='\0'){
		Printf("SN is empty\n");
		return -1;
	}

	strncpy(ct->sn, msg->sn, sizeof(ct->sn));

	Printf("SN:%s\n",ct->sn);
	hlist_add_head(&ct->hashToSn, &rps.devSnHlist[SnHash(ct->sn)]);

	/* 分配7个端口并创建监听, 发送LoginAck */
	if(allocatePortsForDevice(ct)){
		return -1;
	}

	return 0;
}

int recvDevClientData(devClientConn *ct)
{
	int ret=0;
	int msg_len=0;
	rproxy_msg_head *msg=NULL;

	if(sizeof(ct->recvBuf)-ct->recvBufUsed==0){
		return 0;
	}

	ret=recv(ct->devSock,ct->recvBuf+ct->recvBufUsed,sizeof(ct->recvBuf)-ct->recvBufUsed,0);
	if(!ret){
		Printf("Connection closed by peer\n");
		return -1;
	}
	if(ret<0){
		if(errno==EINTR || errno==EWOULDBLOCK || errno==EAGAIN){
			ret=0;
		}else{
			Printf("Connection closed by peer\n");
			return -1;
		}
	}
	ct->recvBufUsed+=ret;

again:
	if(ct->recvBufUsed<(int)sizeof(*msg)){
		goto next;
	}

	msg=(rproxy_msg_head *)ct->recvBuf;

	if(ntohs(msg->magic)!=RPROXY_MSG_MAGIC){
		Printf("head->magic:%04X error\n",ntohs(msg->magic));
		goto err;
	}

	msg_len=ntohs(msg->len);
	if(ct->recvBufUsed<msg_len){
		goto next;
	}

	if(rproxy_checksum_check(ct->recvBuf, msg_len)){
		Printf("checksum error, msgType:%d\n",msg->msgType);
		goto err;
	}

	Printf("msgType:%s\n",msgTypeStr(msg->msgType));
	switch(msg->msgType){
		case msgTypeLogin:
			if(devClientLogin(ct, ct->recvBuf, msg_len)){
				goto err;
			}
			mod_timer(&ct->timer, jiffies+DEV_CONN_CONFIRM_TIMEOUT*HZ);
			break;
		case msgTypeKeepAlive:
			replyKeepAliveAck(ct);
			mod_timer(&ct->timer, jiffies+DEV_CONN_CONFIRM_TIMEOUT*HZ);
			break;
		default:
			Printf("msgType:%d undefined\n",msg->msgType);
			break;
	}

	ct->recvBufUsed-=msg_len;
	if(ct->recvBufUsed){
		memmove(ct->recvBuf,ct->recvBuf+msg_len,ct->recvBufUsed);
		goto again;
	}

next:
	return 0;

err:
	return -1;
}

/* ======================== 代理连接数据转发 ======================== */

int sendDataToProxyClient(proxyClientConn *ct)
{
	int ret=0;

	if(ct==NULL){
		Printf("ct==NULL\n");
		exit(1);
	}
	if(ct->bindCt==NULL){
		Printf("ct->bindCt==NULL\n");
		exit(1);
	}

	if(ct->bindCt->recvBufUsed==0){
		return 0;
	}

	ret=send(ct->proxySock, ct->bindCt->recvBuf, ct->bindCt->recvBufUsed, 0);
	if(!ret){
		Printf("Connection closed by peer\n");
		return -1;
	}
	if(ret<0){
		if(!isIgnoreErrno(errno)){
			Printf("Connection closed by peer\n");
			return -1;
		}
		ret=0;
		epollOutEvent(ct->proxySock,1);
		goto out;
	}

	ct->bindCt->recvBufUsed-=ret;
	if(ct->bindCt->recvBufUsed){
		memmove(ct->bindCt->recvBuf, ct->bindCt->recvBuf+ret, ct->bindCt->recvBufUsed);
		epollOutEvent(ct->proxySock,1);
	}else{
		epollOutEvent(ct->proxySock,0);
	}

out:
	return 0;
}

int sendDataToBindClient(bindClientConn *ct)
{
	int ret=0;

	assert(ct->proxyCt!=NULL);
	ret=send(ct->sock, ct->proxyCt->recvBuf, ct->proxyCt->recvBufUsed, 0);
	if(!ret){
		Printf("Connection closed by peer\n");
		return -1;
	}
	if(ret<0){
		if(!isIgnoreErrno(errno)){
			Printf("Connection closed by peer\n");
			return -1;
		}
		epollOutEvent(ct->sock,1);
		ret=0;
		goto out;
	}

	ct->proxyCt->recvBufUsed-=ret;
	if(ct->proxyCt->recvBufUsed){
		memmove(ct->proxyCt->recvBuf, ct->proxyCt->recvBuf+ret, ct->proxyCt->recvBufUsed);
		epollOutEvent(ct->sock,1);
	}else{
		epollOutEvent(ct->sock,0);
	}

out:
	return 0;
}

/* ======================== 检查代理连接首条消息 ======================== */

int checkProxyClientFirstData(proxyClientConn *ct)
{
	int msg_len=0;
	rproxy_msg_head *msg=NULL;

	if(ct->recvBufUsed<(int)sizeof(*msg)){
		return -1;
	}

	msg=(rproxy_msg_head *)ct->recvBuf;

	if(ntohs(msg->magic)!=RPROXY_MSG_MAGIC){
		Printf("head->magic:%04X error\n",ntohs(msg->magic));
		return -1;
	}

	msg_len=ntohs(msg->len);
	if(ct->recvBufUsed<msg_len){
		return -1;
	}

	if(rproxy_checksum_check(ct->recvBuf, msg_len)){
		Printf("checksum error, msgType:%d\n",msg->msgType);
		return -1;
	}

	Printf("msgType:%s\n",msgTypeStr(msg->msgType));
	if(msg->msgType==msgTypeBindPortAck){
		msg_bind_port_ack_t *bp=(msg_bind_port_ack_t *)ct->recvBuf;
		if(msg_len<(int)sizeof(*bp)){
			Printf("BindPortAck too short\n");
			return -1;
		}
		ct->port=bp->port;  /* 主机序端口号, 标识对应的bindClient */
	}
	else{
		Printf("msgType:%d unexpected\n",msg->msgType);
		return -1;
	}

	ct->recvBufUsed-=msg_len;
	if(ct->recvBufUsed){
		memmove(ct->recvBuf,ct->recvBuf+msg_len,ct->recvBufUsed);
	}

	return 0;
}

int recvBindClientData(bindClientConn *ct);

int recvProxyClientData(proxyClientConn *ct)
{
	int ret=0;

	if(sizeof(ct->recvBuf)-ct->recvBufUsed==0){
		return 0;
	}
	ret=recv(ct->proxySock,ct->recvBuf+ct->recvBufUsed,sizeof(ct->recvBuf)-ct->recvBufUsed,0);
	if(!ret){
		Printf("Connection closed by peer\n");
		return -1;
	}
	if(ret<0){
		if(errno==EINTR || errno==EWOULDBLOCK || errno==EAGAIN){
			ret=0;
		}else{
			Printf("Connection closed by peer\n");
			return -1;
		}
	}
	ct->recvBufUsed+=ret;

	if(ct->state==proxyConnInit){
		if(checkProxyClientFirstData(ct)){
			return -1;
		}

		bindClientConn *bindCt=bindClientFindPort(ct->port);
		if(!bindCt){
			Printf("bindClientFindPort %d faild. close\n",ct->port);
			return -1;
		}
		Printf("bindClientFindPort %d ok\n",ct->port);

		bindCt->proxyCt=ct;
		ct->bindCt=bindCt;
		ct->state=proxyConnConfirm;
		bindCt->state=proxyConnConfirm;
		ct->recvBufUsed=0;

		return sendDataToProxyClient(ct);
	}

	if(ct->state==proxyConnConfirm){
		assert(ct->bindCt!=NULL);
		return sendDataToBindClient(ct->bindCt);
	}

	return 0;
}

int recvBindClientData(bindClientConn *ct)
{
	int ret=0;

	if(sizeof(ct->recvBuf)-ct->recvBufUsed==0){
		return 0;
	}

	ret=recv(ct->sock,ct->recvBuf+ct->recvBufUsed,sizeof(ct->recvBuf)-ct->recvBufUsed,0);
	if(!ret){
		Printf("Connection closed by peer\n");
		return -1;
	}
	if(ret<0){
		if(errno==EINTR || errno==EWOULDBLOCK || errno==EAGAIN){
			ret=0;
		}else{
			Printf("Connection closed by peer\n");
			return -1;
		}
	}
	ct->recvBufUsed+=ret;

	if(ct->state==proxyConnConfirm){
		return sendDataToProxyClient(ct->proxyCt);
	}

	return 0;
}

/* ======================== 删除连接 ======================== */

void proxyClientDelete(proxyClientConn *ct);

void bindClientDelete(bindClientConn *ct)
{
	del_timer(&ct->timer);
	hlist_del(&ct->hashToSock);
	if(!hlist_unhashed(&ct->hashToPort)){
		hlist_del(&ct->hashToPort);
	}
	epollDelete(ct->sock);
	close(ct->sock);

	if(ct->proxyCt){
		ct->proxyCt->bindCt=NULL;
		proxyClientDelete(ct->proxyCt);
		ct->proxyCt=NULL;
	}

	memFree(ct);
}

void devClientDelete(devClientConn *ct)
{
	del_timer(&ct->timer);
	hlist_del(&ct->hashToSock);
	if(!hlist_unhashed(&ct->hashToSn)){
		hlist_del(&ct->hashToSn);
	}
	epollDelete(ct->devSock);
	close(ct->devSock);

	/* 释放所有绑定端口和监听器 */
	releasePortsForDevice(ct);

	memFree(ct);
}

void proxyClientDelete(proxyClientConn *ct)
{
	del_timer(&ct->timer);
	hlist_del(&ct->hashToSock);
	epollDelete(ct->proxySock);
	close(ct->proxySock);

	if(ct->bindCt){
		ct->bindCt->proxyCt=NULL;
		bindClientDelete(ct->bindCt);
		ct->bindCt=NULL;
	}

	memFree(ct);
}

/* ======================== 超时处理 ======================== */

void devClientTimeout(unsigned long data)
{
	devClientDelete((devClientConn *)data);
}

void proxyClientTimeout(unsigned long data)
{
	proxyClientDelete((proxyClientConn *)data);
}

void bindClientTimeout(unsigned long data)
{
	bindClientDelete((bindClientConn *)data);
}

/* ======================== 添加连接 ======================== */

int devClientConnAdd(int sock, struct sockaddr_in *addr)
{
	devClientConn *ct=memMalloc(sizeof(*ct));

	if(!ct){
		Printf("memMalloc failed\n");
		return -1;
	}

	if(epollAdd(sock)){
		goto err;
	}

	ct->ip=addr->sin_addr.s_addr;
	ct->devSock=sock;
	setNonblock(sock);

	INIT_HLIST_NODE(&ct->hashToSn);
	hlist_add_head(&ct->hashToSock, &rps.devSockHlist[sockHash(sock)]);
	setup_timer(&ct->timer, devClientTimeout, (unsigned long)ct);
	mod_timer(&ct->timer, jiffies+DEV_CONN_TIMEOUT*HZ);
	return 0;

err:
	memFree(ct);
	return -1;
}

int proxyClientConnAdd(int sock, struct sockaddr_in *addr)
{
	proxyClientConn *ct=memMalloc(sizeof(*ct));

	if(!ct){
		Printf("memMalloc failed\n");
		return -1;
	}

	if(epollAdd(sock)){
		goto err;
	}

	ct->proxySock=sock;
	ct->state=proxyConnInit;
	setNonblock(sock);

	hlist_add_head(&ct->hashToSock, &rps.proxySockHlist[sockHash(sock)]);
	setup_timer(&ct->timer, proxyClientTimeout, (unsigned long)ct);
	mod_timer(&ct->timer, jiffies+PROXY_CONN_TIMEOUT*HZ);
	return 0;

err:
	memFree(ct);
	return -1;
}

bindClientConn *bindClientConnAdd(int sock, __u16 port, __u8 portType)
{
	bindClientConn *ct=memMalloc(sizeof(*ct));

	if(!ct){
		Printf("memMalloc failed\n");
		return NULL;
	}

	if(epollAdd(sock)){
		goto err;
	}

	ct->port=port;
	ct->portType=portType;
	ct->sock=sock;
	ct->state=proxyConnInit;
	setNonblock(sock);

	hlist_add_head(&ct->hashToPort, &rps.bindPortHlist[portHash(ct->port)]);
	hlist_add_head(&ct->hashToSock, &rps.bindSockHlist[sockHash(sock)]);
	setup_timer(&ct->timer, bindClientTimeout, (unsigned long)ct);
	mod_timer(&ct->timer, jiffies+PROXY_CONN_TIMEOUT*HZ);
	return ct;

err:
	memFree(ct);
	return NULL;
}

/* ======================== APP查询端口处理(异步非阻塞) ======================== */

/*
	APP查询端口短连接, 纯文本协议, 完全异步非阻塞:
	  APP发送:  "QUERY <SN>\n"
	  成功回复: "OK <video0>,<video1>,<audio>,<control>,<videoCtrl>\n"
	  失败回复: "ERR SN_NOT_FOUND\n"
	回复后关闭连接
*/

queryClientConn *queryClientFind(int sock)
{
	struct hlist_node *pos;
	queryClientConn *ct;
	hlist_for_each(pos, &rps.querySockHlist[sockHash(sock)]){
		ct=hlist_entry(pos, queryClientConn, hashToSock);
		if(ct->sock==sock){
			return ct;
		}
	}
	return NULL;
}

void queryClientDelete(queryClientConn *ct)
{
	del_timer(&ct->timer);
	hlist_del(&ct->hashToSock);
	epollDelete(ct->sock);
	close(ct->sock);
	memFree(ct);
}

void queryClientTimeout(unsigned long data)
{
	queryClientDelete((queryClientConn *)data);
}

int queryClientConnAdd(int sock)
{
	queryClientConn *ct=memMalloc(sizeof(*ct));

	if(!ct){
		Printf("memMalloc failed\n");
		return -1;
	}

	if(epollAdd(sock)){
		goto err;
	}

	ct->sock=sock;
	setNonblock(sock);

	INIT_HLIST_NODE(&ct->hashToSock);
	hlist_add_head(&ct->hashToSock, &rps.querySockHlist[sockHash(sock)]);
	setup_timer(&ct->timer, queryClientTimeout, (unsigned long)ct);
	mod_timer(&ct->timer, jiffies+QUERY_CONN_TIMEOUT*HZ);
	return 0;

err:
	memFree(ct);
	return -1;
}

/*
	处理已接收到的查询请求数据, 检查是否包含完整请求(\n结尾)
	如果完整则解析并构建响应到sendBuf, 触发发送
	返回: 0=正常, -1=出错/删除连接
*/
int processQueryRequest(queryClientConn *ct)
{
	char *sn, *p;
	int len;
	devClientConn *devCt;

	/* 检查是否收到完整行(包含\n) */
	p=memchr(ct->recvBuf, '\n', ct->recvBufUsed);
	if(!p){
		/* 还没收到完整行, 继续等待 */
		if(ct->recvBufUsed>=(int)sizeof(ct->recvBuf)-1){
			Printf("QueryPort recvBuf overflow\n");
			return -1;
		}
		return 0;
	}

	/* 去掉末尾的 \n \r 空格 */
	*p='\0';
	while(p>ct->recvBuf && (*(p-1)=='\r' || *(p-1)==' ')){
		*(--p)='\0';
	}

	Printf("QueryPort recv: [%s]\n",ct->recvBuf);

	/* 解析: "QUERY <SN>" */
	if(strncmp(ct->recvBuf, "QUERY ", 6)!=0){
		len=snprintf(ct->sendBuf, sizeof(ct->sendBuf), "ERR BAD_REQUEST\n");
		ct->sendBufUsed=len;
		epollOutEvent(ct->sock, 1);
		return 0;
	}

	sn=ct->recvBuf+6;
	while(*sn==' ') sn++;

	if(*sn=='\0'){
		len=snprintf(ct->sendBuf, sizeof(ct->sendBuf), "ERR SN_EMPTY\n");
		ct->sendBufUsed=len;
		epollOutEvent(ct->sock, 1);
		return 0;
	}

	Printf("QueryPort SN:%s\n",sn);

	devCt=devClientConnFindBySn(sn);
	if(!devCt){
		Printf("QueryPort SN:%s not found\n",sn);
		len=snprintf(ct->sendBuf, sizeof(ct->sendBuf), "ERR SN_NOT_FOUND\n");
	}else{
		Printf("QueryPort SN:%s found, ports: %d,%d,%d,%d,%d,%d,%d\n",
			sn,
			devCt->bind[0].port,
			devCt->bind[1].port,
			devCt->bind[2].port,
			devCt->bind[3].port,
			devCt->bind[4].port,
			devCt->bind[5].port,
			devCt->bind[6].port);
		len=snprintf(ct->sendBuf, sizeof(ct->sendBuf), "OK %d,%d,%d,%d,%d,%d,%d\n",
			devCt->bind[0].port,
			devCt->bind[1].port,
			devCt->bind[2].port,
			devCt->bind[3].port,
			devCt->bind[4].port,
			devCt->bind[5].port,
			devCt->bind[6].port);
	}

	ct->sendBufUsed=len;
	epollOutEvent(ct->sock, 1);
	return 0;
}

int recvQueryClientData(queryClientConn *ct)
{
	int ret;

	if(sizeof(ct->recvBuf)-ct->recvBufUsed==0){
		Printf("QueryPort recvBuf full\n");
		return -1;
	}

	ret=recv(ct->sock, ct->recvBuf+ct->recvBufUsed,
		sizeof(ct->recvBuf)-ct->recvBufUsed-1, 0);
	if(!ret){
		Printf("QueryPort connection closed by peer\n");
		return -1;
	}
	if(ret<0){
		if(errno==EINTR || errno==EWOULDBLOCK || errno==EAGAIN){
			return 0;
		}
		Printf("QueryPort connection closed by peer\n");
		return -1;
	}
	ct->recvBufUsed+=ret;
	ct->recvBuf[ct->recvBufUsed]='\0';

	return processQueryRequest(ct);
}

int sendQueryClientData(queryClientConn *ct)
{
	int ret;

	if(ct->sendBufUsed==0){
		return 0;
	}

	ret=send(ct->sock, ct->sendBuf, ct->sendBufUsed, 0);
	if(!ret){
		return -1;
	}
	if(ret<0){
		if(!isIgnoreErrno(errno)){
			return -1;
		}
		return 0;
	}

	ct->sendBufUsed-=ret;
	if(ct->sendBufUsed){
		memmove(ct->sendBuf, ct->sendBuf+ret, ct->sendBufUsed);
	}else{
		/* 响应发送完毕, 关闭连接 */
		epollOutEvent(ct->sock, 0);
		return -1;  /* 返回-1让调用者删除连接 */
	}

	return 0;
}

/* ======================== 监听端口初始化 ======================== */

int initDevListener()
{
	struct sockaddr_in my_addr;
	int optval=1;

	rps.devSock=socket(AF_INET,SOCK_STREAM,0);
	if(rps.devSock==-1){
		Printf("socket: %s\n",strerror(errno));
		return -1;
	}

	bzero(&my_addr, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = rps.devPort;
	my_addr.sin_addr.s_addr = INADDR_ANY;

	setsockopt(rps.devSock, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(optval));
	if(bind(rps.devSock, (struct sockaddr *)&my_addr, sizeof(struct sockaddr))==-1){
		Printf("Dev bind: %s\n",strerror(errno));
		return -1;
	}

	if(listen(rps.devSock, SOMAXCONN)==-1){
		Printf("listen: %s\n",strerror(errno));
		return -1;
	}
	setNonblock(rps.devSock);
	if(epollAdd(rps.devSock)){
		return -1;
	}

	return 0;
}

int initProxyListener()
{
	struct sockaddr_in my_addr;
	int optval=1;

	rps.proxySock=socket(AF_INET,SOCK_STREAM,0);
	if(rps.proxySock==-1){
		Printf("socket: %s\n",strerror(errno));
		return -1;
	}

	bzero(&my_addr, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = rps.proxyPort;
	my_addr.sin_addr.s_addr = INADDR_ANY;

	setsockopt(rps.proxySock, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(optval));
	if(bind(rps.proxySock, (struct sockaddr *)&my_addr, sizeof(struct sockaddr))==-1){
		Printf("Proxy bind: %s\n",strerror(errno));
		return -1;
	}

	if(listen(rps.proxySock, SOMAXCONN)==-1){
		Printf("listen: %s\n",strerror(errno));
		return -1;
	}
	setNonblock(rps.proxySock);
	if(epollAdd(rps.proxySock)){
		return -1;
	}

	return 0;
}

int initQueryListener()
{
	struct sockaddr_in my_addr;
	int optval=1;

	rps.querySock=socket(AF_INET,SOCK_STREAM,0);
	if(rps.querySock==-1){
		Printf("socket: %s\n",strerror(errno));
		return -1;
	}

	bzero(&my_addr, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = rps.queryPort;
	my_addr.sin_addr.s_addr = INADDR_ANY;

	setsockopt(rps.querySock, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(optval));
	if(bind(rps.querySock, (struct sockaddr *)&my_addr, sizeof(struct sockaddr))==-1){
		Printf("Query bind: %s\n",strerror(errno));
		return -1;
	}

	if(listen(rps.querySock, SOMAXCONN)==-1){
		Printf("listen: %s\n",strerror(errno));
		return -1;
	}
	setNonblock(rps.querySock);
	if(epollAdd(rps.querySock)){
		return -1;
	}

	return 0;
}

/* ======================== 主事件循环 ======================== */

int runEpoll(int timeout)
{
	int n,i,waitNum;
	socklen_t len=0;
	int newSock;
	struct sockaddr_in addr;
	struct epoll_event events[EPOLL_MAX_NUM];

	bzero(events, sizeof(struct epoll_event)*EPOLL_MAX_NUM);
	if(rps.epollMaxEvents>EPOLL_MAX_NUM){
		waitNum=EPOLL_MAX_NUM;
	}else{
		waitNum=rps.epollMaxEvents;
	}
	n=epoll_wait(rps.epollFd,events,waitNum,timeout);
	if(n<1){
		return 0;
	}

	for(i=0;i<n;i++){
		if(events[i].data.fd==0){
			continue;
		}
		if(events[i].data.fd==rps.devSock && events[i].events&EPOLLIN){
			while(1){
				len=sizeof(struct sockaddr_in);
				newSock=accept(rps.devSock, (struct sockaddr *)&addr, &len);
				if(newSock<0){
					if(errno==EMFILE || errno==ENFILE){
						Printf("accept dev: too many open files, pause 500ms\n");
						usleep(500000);
					}
					break;
				}else{
					if(devClientConnAdd(newSock,&addr)){
						close(newSock);
					}
				}
			}
		}else if(events[i].data.fd==rps.proxySock && events[i].events&EPOLLIN){
			while(1){
				len=sizeof(struct sockaddr_in);
				newSock=accept(rps.proxySock, (struct sockaddr *)&addr, &len);
				if(newSock<0){
					if(errno==EMFILE || errno==ENFILE){
						Printf("accept proxy: too many open files, pause 500ms\n");
						usleep(500000);
					}
					break;
				}else{
					if(proxyClientConnAdd(newSock,&addr)){
						close(newSock);
					}
				}
			}
		}else if(events[i].data.fd==rps.querySock && events[i].events&EPOLLIN){
			/* APP查询端口: accept后加入epoll, 异步处理 */
			while(1){
				len=sizeof(struct sockaddr_in);
				newSock=accept(rps.querySock, (struct sockaddr *)&addr, &len);
				if(newSock<0){
					if(errno==EMFILE || errno==ENFILE){
						Printf("accept query: too many open files, pause 500ms\n");
						usleep(500000);
					}
					break;
				}else{
					Printf("QueryPort connection from %s\n",
						ipstr(addr.sin_addr.s_addr));
					if(queryClientConnAdd(newSock)){
						close(newSock);
					}
				}
			}
		}else{
			devClientConn *devCt;
			proxyClientConn *proxyCt;
			bindClientConn *bindCt;
			bindListenerConn *lt;
			queryClientConn *queryCt;

			devCt=devClientFind(events[i].data.fd);
			if(devCt){
				if(events[i].events&EPOLLIN){
					if(recvDevClientData(devCt)){
						Printf("devClientDelete\n");
						devClientDelete(devCt);
					}
				}else if(events[i].events&EPOLLOUT){
					sendDataToDevClient(devCt,NULL,0);
				}else{
					Printf("devClientDelete\n");
					devClientDelete(devCt);
				}
				continue;
			}

			proxyCt=proxyClientFind(events[i].data.fd);
			if(proxyCt){
				if(events[i].events&EPOLLIN){
					if(recvProxyClientData(proxyCt)){
						proxyClientDelete(proxyCt);
					}
				}else if(events[i].events&EPOLLOUT){
					sendDataToProxyClient(proxyCt);
				}else{
					proxyClientDelete(proxyCt);
				}
				continue;
			}

			bindCt=bindPortClientFind(events[i].data.fd);
			if(bindCt){
				if(events[i].events&EPOLLIN){
					if(recvBindClientData(bindCt)){
						bindClientDelete(bindCt);
					}
				}else if(events[i].events&EPOLLOUT){
					sendDataToBindClient(bindCt);
				}else{
					bindClientDelete(bindCt);
				}
				continue;
			}

			queryCt=queryClientFind(events[i].data.fd);
			if(queryCt){
				if(events[i].events&EPOLLIN){
					if(recvQueryClientData(queryCt)){
						queryClientDelete(queryCt);
					}
				}else if(events[i].events&EPOLLOUT){
					if(sendQueryClientData(queryCt)){
						queryClientDelete(queryCt);
					}
				}else{
					queryClientDelete(queryCt);
				}
				continue;
			}

			/* 检查是否是绑定监听端口有新连接 */
			lt=bindListenerFind(events[i].data.fd);
			if(lt){
				while(1){
					len=sizeof(struct sockaddr_in);
					newSock=accept(lt->sock, (struct sockaddr *)&addr, &len);
					if(newSock<0){
						if(errno==EMFILE || errno==ENFILE){
							Printf("accept bind(%d): too many open files, pause 500ms\n",lt->port);
							usleep(500000);
						}
						break;
					}else{
						devCt=lt->devCt;
						bindCt=bindClientConnAdd(newSock, lt->port, lt->portType);
						if(bindCt){
							Printf("new %s connection on port %d from %s\n",
								portTypeStr(lt->portType), lt->port,
								ipstr(addr.sin_addr.s_addr));
							/* 通知设备有新的外部连接 */
							if(sendBindPortNotify(devCt, lt->portType, lt->port)){
								bindClientDelete(bindCt);
							}
						}else{
							close(newSock);
						}
					}
				}
				continue;
			}

			Printf("Not find Client\n");
			close(events[i].data.fd);
			epollDelete(events[i].data.fd);
		}
	}

	return 0;
}

/* ======================== 配置文件 ======================== */

int checkConfigFile(void)
{
	FILE *fp=NULL;
	fp=fopen(rps.configFile,"r");
	if(!fp){
		Printf("fopen %s failed:%s\n",rps.configFile,strerror(errno));
		return -1;
	}
	fclose(fp);
	return 0;
}

char *getConfigName(char *name)
{
	static char buf[1024]={0};
	char *retstr="";
	FILE *fp=NULL;
	char *p;

	if(!name) goto out;

	fp=fopen(rps.configFile,"r");
	if(!fp) goto out;

	while(1){
		memset(buf, 0, sizeof(buf));
		if(fgets(buf,sizeof(buf),fp)==NULL){
			break;
		}
		if(buf[0]=='#') continue;
		if(strncmp(buf, name, strlen(name))==0){
			p=buf+strlen(name);
			if(p[0]=='='){
				if(p[strlen(p)-1]=='\n') p[strlen(p)-1]=0;
				if(p[strlen(p)-1]=='\r') p[strlen(p)-1]=0;
				retstr=p+1;
				goto out;
			}
		}
	}

out:
	if(fp) fclose(fp);
	return retstr;
}

int initConfig()
{
	char *val;
	int rangeStart, rangeEnd;

	rps.devPort=htons(atoi(getConfigName("devPort")));
	if(!rps.devPort){
		Printf("devPort Undefined\n");
		return -1;
	}
	rps.proxyPort=htons(atoi(getConfigName("proxyPort")));
	if(!rps.proxyPort){
		Printf("proxyPort Undefined\n");
		return -1;
	}
	rps.queryPort=htons(atoi(getConfigName("queryPort")));
	if(!rps.queryPort){
		Printf("queryPort Undefined\n");
		return -1;
	}

	/* 端口范围 */
	val=getConfigName("portRangeStart");
	if(!val[0]){
		Printf("portRangeStart Undefined\n");
		return -1;
	}
	rangeStart=atoi(val);
	val=getConfigName("portRangeEnd");
	if(!val[0]){
		Printf("portRangeEnd Undefined\n");
		return -1;
	}
	rangeEnd=atoi(val);

	if(rangeStart>=rangeEnd || rangeStart<1 || rangeEnd>65535){
		Printf("Invalid portRange: %d-%d\n",rangeStart,rangeEnd);
		return -1;
	}

	if(initPortPool(rangeStart, rangeEnd)){
		return -1;
	}

	Printf("Port pool: %d-%d (%d ports)\n",rangeStart,rangeEnd,rangeEnd-rangeStart);

	return 0;
}

/* ======================== main ======================== */

int main(int argc, char **argv)
{
	initTimer();
	initSignalHandler();
	initList();

	if(argc<2){
		printf("Usage %s [configFile] (debug)\n",argv[0]);
		return -1;
	}

	strncpy(rps.configFile, argv[1], sizeof(rps.configFile));
	if(argc==3){
		rps.debug=atoi(argv[2]);
	}

	if(checkConfigFile()){
		return -1;
	}

	if(initConfig()){
		return -1;
	}

	if(epollInit()){
		return -1;
	}

	/* 监听设备端口 */
	if(initDevListener()){
		return -1;
	}

	/* 监听代理连接端口 */
	if(initProxyListener()){
		return -1;
	}

	/* 监听APP查询端口 */
	if(initQueryListener()){
		return -1;
	}

	Printf("Server init ok\n");
	while(1){
		runEpoll(100);
		run_timers();
	}

	releasePortPool();

	return 0;
}
