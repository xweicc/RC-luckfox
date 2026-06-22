// Copyright 2022 Rockchip Electronics Co., Ltd. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "video.h"
#include "audio.h"
#include "rockiva.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <poll.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <dirent.h>
#include <rk_mpi_vdec.h>

#define HAS_VO 0
#if HAS_VO
#include <rk_mpi_vo.h>
#endif

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "video.c"

#define RKISP_MAINPATH 0
#define RKISP_SELFPATH 1
#define RKISP_FBCPATH 2
#define VIDEO_PIPE_0 0
#define VIDEO_PIPE_1 1
#define VIDEO_PIPE_2 2
#define VIDEO_PIPE_3 3
#define JPEG_VENC_CHN 4
#define DRAW_NN_VENC_CHN_ID 0
#define VPSS_ROTATE 6
#define VPSS_BGR 0
#define DRAW_NN_OSD_ID 7
#define RED_COLOR 0x0000FF
#define BLUE_COLOR 0xFF0000

#define RK3588_VO_DEV_HDMI 0
#define RK3588_VO_DEV_MIPI 3
#define RK3588_VOP_LAYER_CLUSTER0 0

#define RTSP_URL_0 "/live/0"
#define RTSP_URL_1 "/live/1"
#define RTSP_URL_2 "/live/2"
#define RTMP_URL_0 "rtmp://127.0.0.1:1935/live/mainstream"
#define RTMP_URL_1 "rtmp://127.0.0.1:1935/live/substream"
#define RTMP_URL_2 "rtmp://127.0.0.1:1935/live/thirdstream"

#define TCP_VIDEO_PORT_0 5100
#define TCP_VIDEO_PORT_1 5101
#define TCP_VIDEO_PORT_2 5105
#define TCP_CONTROL_PORT 5104
#define TCP_VIDEO_STREAM_COUNT 3

// USB Camera
#define USB_VENC_CHN VIDEO_PIPE_3
#define VDEC_CHN_ID  0
#define USB_CAMERA_DEFAULT_WIDTH  1280
#define USB_CAMERA_DEFAULT_HEIGHT 720
#define USB_CAMERA_DEFAULT_FPS    30
#define USB_CAMERA_BUF_COUNT      4

static int get_jpeg_cnt = 0;
static int enable_ivs, enable_jpeg, enable_venc_0, enable_venc_1, enable_rtsp, enable_rtmp, enable_tcp_video;
static int g_enable_vo, g_vo_dev_id, g_vi_chn_id, enable_npu, enable_wrap, enable_osd;
static int g_video_run_ = 1;
static int pipe_id_ = 0;
static int dev_id_ = 0;
static int g_nn_osd_run_ = 0;
static int cycle_snapshot_flag = 0;
static const char *tmp_output_data_type = "H.264";
static const char *tmp_rc_mode;
static const char *tmp_h264_profile;
static const char *tmp_smart;
static const char *tmp_gop_mode;
static const char *tmp_rc_quality;
static pthread_t vi_thread_1, venc_thread_0, venc_thread_1, venc_thread_2, jpeg_venc_thread_id,
    vpss_thread_rgb, cycle_snapshot_thread_id, get_nn_update_osd_thread_id, get_vi_2_send_thread,
    get_ivs_result_thread;
static pthread_t tcp_video_thread_0, tcp_video_thread_1, tcp_video_thread_2;
static volatile int tcp_video_client_fd[TCP_VIDEO_STREAM_COUNT] = {-1, -1, -1};
static int tcp_video_server_fd[TCP_VIDEO_STREAM_COUNT] = {-1, -1, -1};
static pthread_t tcp_control_thread;
static volatile int tcp_control_client_fd = -1;
static int tcp_control_server_fd = -1;
// Lazy VENC bind state
static volatile int pipe_0_bound = 0, pipe_1_bound = 0;
// USB Camera state
static int usb_camera_fd_ = -1;
static int usb_camera_available_ = 0;
static int usb_camera_streaming_ = 0;
static int usb_cam_width_ = USB_CAMERA_DEFAULT_WIDTH;
static int usb_cam_height_ = USB_CAMERA_DEFAULT_HEIGHT;
static int usb_cam_fps_ = USB_CAMERA_DEFAULT_FPS;
static void *usb_v4l2_buffers_[USB_CAMERA_BUF_COUNT];
static size_t usb_v4l2_buffer_lengths_[USB_CAMERA_BUF_COUNT];
static pthread_t usb_camera_thread_;

// Forward declarations
static int tcp_video_send_frame(int stream_id, void *data, unsigned int len);
static int usb_camera_init(void);
static void usb_camera_deinit(void);
static void *usb_camera_encode_thread(void *arg);

static MPP_CHN_S vi_chn, vpss_bgr_chn, vpss_rotate_chn, vo_chn, vpss_out_chn[4], venc_chn, ivs_chn;
static VO_DEV VoLayer = RK3588_VOP_LAYER_CLUSTER0;
typedef enum rkCOLOR_INDEX_E {
	RGN_COLOR_LUT_INDEX_0 = 0,
	RGN_COLOR_LUT_INDEX_1 = 1,
} COLOR_INDEX_E;

#if HAS_VO
static void *get_vi_send_vo(void *arg) {
	LOG_DEBUG("#Start %s thread, arg:%p\n", __func__, arg);
	prctl(PR_SET_NAME, "get_vi_send_vo", 0, 0, 0);
	VIDEO_FRAME_INFO_S stViFrame;
	VI_CHN_STATUS_S stChnStatus;
	int loopCount = 0;
	int ret = 0;

	while (g_video_run_) {
		// 5.get the frame
		ret = RK_MPI_VI_GetChnFrame(pipe_id_, VIDEO_PIPE_1, &stViFrame, 1000);
		if (ret == RK_SUCCESS) {
			void *data = RK_MPI_MB_Handle2VirAddr(stViFrame.stVFrame.pMbBlk);
			LOG_ERROR("RK_MPI_VI_GetChnFrame ok:data %p loop:%d pts:%" PRId64 " ms\n", data,
			          loopCount, stViFrame.stVFrame.u64PTS / 1000);
			// 6.get the channel status
			// ret = RK_MPI_VI_QueryChnStatus(pipe_id_, VIDEO_PIPE_1, &stChnStatus);
			// LOG_ERROR("RK_MPI_VI_QueryChnStatus ret %x, "
			//           "w:%d,h:%d,enable:%d,lost:%d,framerate:%d,vbfail:%d\n",
			//           ret, stChnStatus.stSize.u32Width, stChnStatus.stSize.u32Height,
			//           stChnStatus.bEnable, stChnStatus.u32LostFrame, stChnStatus.u32FrameRate,
			//           stChnStatus.u32VbFail);

			// send vo
			ret = RK_MPI_VO_SendFrame(VoLayer, 0, &stViFrame, 1000);
			if (ret)
				LOG_ERROR("RK_MPI_VO_SendFrame timeout %x\n", ret);

			// 7.release the frame
			ret = RK_MPI_VI_ReleaseChnFrame(pipe_id_, VIDEO_PIPE_1, &stViFrame);
			if (ret != RK_SUCCESS) {
				LOG_ERROR("RK_MPI_VI_ReleaseChnFrame fail %x\n", ret);
			}
			loopCount++;
		} else {
			LOG_ERROR("RK_MPI_VI_GetChnFrame timeout %x\n", ret);
		}
		usleep(10 * 1000);
	}

	return 0;
}
#endif

// Lazy VENC encoding: only bind VI->VENC and encode when TCP client is connected
static void *rkipc_get_venc_lazy(void *arg) {
	int stream_id = (int)(intptr_t)arg;
	char thread_name[16];
	snprintf(thread_name, sizeof(thread_name), "RkipcVenc%d", stream_id);
	prctl(PR_SET_NAME, thread_name, 0, 0, 0);
	LOG_INFO("Lazy VENC thread started for stream %d\n", stream_id);

	VENC_STREAM_S stFrame;
	int ret = 0;
	int bound = 0;
	stFrame.pstPack = malloc(sizeof(VENC_PACK_S));

	// Local channel structs for bind/unbind (thread-safe, no shared state)
	MPP_CHN_S local_vi, local_venc;
	local_vi.enModId = RK_ID_VI;
	local_vi.s32DevId = 0;
	local_vi.s32ChnId = stream_id;
	local_venc.enModId = RK_ID_VENC;
	local_venc.s32DevId = 0;
	local_venc.s32ChnId = stream_id;

	volatile int *bound_flag = (stream_id == 0) ? &pipe_0_bound : &pipe_1_bound;

	while (g_video_run_) {
		int client_connected = (__atomic_load_n(&tcp_video_client_fd[stream_id], __ATOMIC_ACQUIRE) >= 0);

		if (client_connected && !bound) {
			// TCP client connected: bind VI -> VENC to start encoding
			ret = RK_MPI_SYS_Bind(&local_vi, &local_venc);
			if (ret) {
				LOG_ERROR("VENC.%d: Bind VI->VENC failed ret=%#x\n", stream_id, ret);
				usleep(100000);
				continue;
			}
			bound = 1;
			*bound_flag = 1;
			// Reset VENC to clear stale buffers and force IDR on first frame
			RK_MPI_VENC_ResetChn(stream_id);
			RK_MPI_VENC_RequestIDR(stream_id, RK_TRUE);
			LOG_INFO("VENC.%d: VI->VENC bound, encoding started (IDR requested)\n", stream_id);
		} else if (!client_connected && bound) {
			// TCP client disconnected: unbind to stop encoding
			ret = RK_MPI_SYS_UnBind(&local_vi, &local_venc);
			if (ret)
				LOG_ERROR("VENC.%d: Unbind VI->VENC failed ret=%#x\n", stream_id, ret);
			bound = 0;
			*bound_flag = 0;
			LOG_INFO("VENC.%d: VI->VENC unbound, encoding stopped\n", stream_id);
			usleep(50000);
			continue;
		} else if (!bound) {
			// No client, not bound: sleep and check again
			usleep(50000);
			continue;
		}

		// Get encoded frame from VENC
		ret = RK_MPI_VENC_GetStream(stream_id, &stFrame, 2500);
		if (ret == RK_SUCCESS) {
			void *data = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
			// Send to TCP client
			if (enable_tcp_video)
				tcp_video_send_frame(stream_id, data, stFrame.pstPack->u32Len);
			// Write to storage
			int is_key = (stFrame.pstPack->DataType.enH265EType == H265E_NALU_IDRSLICE) ||
			             (stFrame.pstPack->DataType.enH265EType == H265E_NALU_ISLICE) ||
			             (stFrame.pstPack->DataType.enH264EType == H264E_NALU_IDRSLICE) ||
			             (stFrame.pstPack->DataType.enH264EType == H264E_NALU_ISLICE);
			rk_storage_write_video_frame(stream_id, data, stFrame.pstPack->u32Len,
			                             stFrame.pstPack->u64PTS, is_key);

			ret = RK_MPI_VENC_ReleaseStream(stream_id, &stFrame);
			if (ret != RK_SUCCESS)
				LOG_ERROR("RK_MPI_VENC_ReleaseStream fail %x\n", ret);
		} else {
			LOG_ERROR("VENC.%d: GetStream error ret=%#x\n", stream_id, ret);
		}
	}

	// Cleanup: unbind if still bound
	if (bound) {
		RK_MPI_SYS_UnBind(&local_vi, &local_venc);
		*bound_flag = 0;
	}
	if (stFrame.pstPack)
		free(stFrame.pstPack);

	return 0;
}

static int rga_nv12_border(rga_buffer_t buf, int x, int y, int width, int height, int line_pixel,
                           int color) {
	im_rect rect_up = {x, y, width, line_pixel};
	im_rect rect_buttom = {x, y + height - line_pixel, width, line_pixel};
	im_rect rect_left = {x, y, line_pixel, height};
	im_rect rect_right = {x + width - line_pixel, y, line_pixel, height};
	IM_STATUS STATUS = imfill(buf, rect_up, color);
	STATUS |= imfill(buf, rect_buttom, color);
	STATUS |= imfill(buf, rect_left, color);
	STATUS |= imfill(buf, rect_right, color);
	return STATUS == IM_STATUS_SUCCESS ? 0 : 1;
}

static void *rkipc_get_vi_draw_send_venc(void *arg) {
	LOG_DEBUG("#Start %s thread, arg:%p\n", __func__, arg);
	prctl(PR_SET_NAME, "RkipcVi2Venc", 0, 0, 0);
	VIDEO_FRAME_INFO_S stViFrame;
	VI_CHN_STATUS_S stChnStatus;
	int loopCount = 0;
	int ret = 0;
	int line_pixel = 2;
	long long last_ba_result_time;
	RockIvaBaResult ba_result;
	im_handle_param_t param;
	RockIvaBaObjectInfo *object;
	rga_buffer_handle_t handle;
	rga_buffer_t src;

	memset(&ba_result, 0, sizeof(ba_result));
	memset(&param, 0, sizeof(im_handle_param_t));
	while (g_video_run_) {
		// 5.get the frame
		ret = RK_MPI_VI_GetChnFrame(pipe_id_, VIDEO_PIPE_1, &stViFrame, 1000);
		if (ret == RK_SUCCESS) {
			uint64_t phy_data = RK_MPI_MB_Handle2PhysAddr(stViFrame.stVFrame.pMbBlk);
			// LOG_DEBUG("phy_data %p, loop:%d pts:%" PRId64 " ms\n", phy_data, loopCount,
			//           stViFrame.stVFrame.u64PTS / 1000);

			ret = rkipc_rknn_object_get(&ba_result);
			if ((!ret && ba_result.objNum) ||
			    ((ret == -1) && (rkipc_get_curren_time_ms() - last_ba_result_time < 300))) {
				// LOG_DEBUG("ret is %d, ba_result.objNum is %d\n", ret, ba_result.objNum);
				handle = importbuffer_physicaladdr(phy_data, &param);
				src = wrapbuffer_handle_t(handle, stViFrame.stVFrame.u32Width,
				                          stViFrame.stVFrame.u32Height, stViFrame.stVFrame.u32Width,
				                          stViFrame.stVFrame.u32Height, RK_FORMAT_YCbCr_420_SP);
				if (!ret)
					last_ba_result_time = rkipc_get_curren_time_ms();
				for (int i = 0; i < ba_result.objNum; i++) {
					int x, y, w, h;
					object = &ba_result.triggerObjects[i];
					LOG_DEBUG("topLeft:[%d,%d], bottomRight:[%d,%d],"
					          "objId is %d, frameId is %d, score is %d, type is %d\n",
					          object->objInfo.rect.topLeft.x, object->objInfo.rect.topLeft.y,
					          object->objInfo.rect.bottomRight.x,
					          object->objInfo.rect.bottomRight.y, object->objInfo.objId,
					          object->objInfo.frameId, object->objInfo.score, object->objInfo.type);
					x = stViFrame.stVFrame.u32Width * object->objInfo.rect.topLeft.x / 10000;
					y = stViFrame.stVFrame.u32Height * object->objInfo.rect.topLeft.y / 10000;
					w = stViFrame.stVFrame.u32Width *
					    (object->objInfo.rect.bottomRight.x - object->objInfo.rect.topLeft.x) /
					    10000;
					h = stViFrame.stVFrame.u32Height *
					    (object->objInfo.rect.bottomRight.y - object->objInfo.rect.topLeft.y) /
					    10000;
					x = x / 2 * 2;
					y = y / 2 * 2;
					w = w / 2 * 2;
					h = h / 2 * 2;
					while (x + w + line_pixel >= stViFrame.stVFrame.u32Width) {
						w -= 8;
					}
					while (y + h + line_pixel >= stViFrame.stVFrame.u32Height) {
						h -= 8;
					}
					LOG_DEBUG("i is %d, x,y,w,h is %d,%d,%d,%d\n", i, x, y, w, h);
					rga_nv12_border(src, x, y, w, h, line_pixel, 0x000000ff);
					// LOG_INFO("draw rect time-consuming is %lld\n",(rkipc_get_curren_time_ms() -
					// last_ba_result_time));
					// LOG_INFO("triggerRules is %d, ruleID is %d, triggerType is %d\n",
					//          object->triggerRules,
					//          object->firstTrigger.ruleID,
					//          object->firstTrigger.triggerType);
				}
				releasebuffer_handle(handle);
			}

			// send venc
			ret = RK_MPI_VENC_SendFrame(VIDEO_PIPE_1, &stViFrame, 1000);
			if (ret)
				LOG_ERROR("RK_MPI_VENC_SendFrame timeout %x\n", ret);
			// 7.release the frame
			ret = RK_MPI_VI_ReleaseChnFrame(pipe_id_, VIDEO_PIPE_1, &stViFrame);
			if (ret != RK_SUCCESS) {
				LOG_ERROR("RK_MPI_VI_ReleaseChnFrame fail %x\n", ret);
			}

			loopCount++;
		} else {
			LOG_ERROR("RK_MPI_VI_GetChnFrame timeout %x\n", ret);
		}
	}

	return 0;
}

// rkipc_get_venc_1 used by NPU draw path (rkipc_get_vi_draw_send_venc sends to VENC.1)
// For TCP lazy encoding, we reuse rkipc_get_venc_lazy with stream_id=1

// ========== TCP Video Server ==========
static void *tcp_video_accept_thread(void *arg) {
	int stream_id = (int)(intptr_t)arg;
	char thread_name[16];
	snprintf(thread_name, sizeof(thread_name), "TcpVideo%d", stream_id);
	prctl(PR_SET_NAME, thread_name, 0, 0, 0);
	LOG_INFO("TCP video accept thread started for stream %d on port %d\n", stream_id,
	         stream_id == 0 ? TCP_VIDEO_PORT_0 : (stream_id == 1 ? TCP_VIDEO_PORT_1 : TCP_VIDEO_PORT_2));

	while (g_video_run_) {
		struct sockaddr_in client_addr;
		socklen_t addr_len = sizeof(client_addr);
		int client_fd = accept(tcp_video_server_fd[stream_id],
		                       (struct sockaddr *)&client_addr, &addr_len);
		if (client_fd < 0) {
			if (g_video_run_ && errno != EINTR)
				LOG_ERROR("TCP video accept error on stream %d: %s\n", stream_id, strerror(errno));
			continue;
		}

		// Set TCP_NODELAY for low latency
		int flag = 1;
		setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
		// Set send buffer size
		int sndbuf = 512 * 1024;
		setsockopt(client_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

		// Disconnect old connection, keep only the new one
		int old_fd = __atomic_exchange_n(&tcp_video_client_fd[stream_id], client_fd, __ATOMIC_SEQ_CST);
		if (old_fd >= 0) {
			LOG_INFO("TCP video.%d: disconnecting old client\n", stream_id);
			close(old_fd);
		}
		LOG_INFO("TCP video.%d: new client connected from %s:%d\n", stream_id,
		         inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
	}
	return NULL;
}

static int tcp_video_send_frame(int stream_id, void *data, unsigned int len) {
	int fd = __atomic_load_n(&tcp_video_client_fd[stream_id], __ATOMIC_ACQUIRE);
	if (fd < 0)
		return 0;

	// Use poll with timeout to avoid blocking VENC thread when client is slow
	struct pollfd pfd;
	pfd.fd = fd;
	pfd.events = POLLOUT;
	int ret = poll(&pfd, 1, 100); // 100ms timeout
	if (ret <= 0) {
		// Timeout or error, disconnect client to avoid blocking pipeline
		LOG_INFO("TCP video.%d client too slow or disconnected (poll ret=%d, errno=%d: %s)\n",
		         stream_id, ret, errno, strerror(errno));
		int expected = fd;
		__atomic_compare_exchange_n(&tcp_video_client_fd[stream_id], &expected, -1,
		                            0, __ATOMIC_SEQ_CST, __ATOMIC_RELAXED);
		if (expected == fd)
			close(fd);
		return -1;
	}

	// Send raw H.265 Annex B stream directly (no length header)
	// Client splits NAL units by scanning start code 0x00000001
	ssize_t written = send(fd, data, len, MSG_DONTWAIT | MSG_NOSIGNAL);
	if (written < (ssize_t)len) {
		// Send failed, client disconnected
		LOG_INFO("TCP video.%d client disconnected (send ret=%zd, errno=%d: %s)\n",
		         stream_id, written, errno, strerror(errno));
		int expected = fd;
		__atomic_compare_exchange_n(&tcp_video_client_fd[stream_id], &expected, -1,
		                            0, __ATOMIC_SEQ_CST, __ATOMIC_RELAXED);
		if (expected == fd)
			close(fd);
		return -1;
	}
	return 0;
}

static int tcp_video_server_init() {
	static const int ports[] = {TCP_VIDEO_PORT_0, TCP_VIDEO_PORT_1, TCP_VIDEO_PORT_2};
	for (int i = 0; i < TCP_VIDEO_STREAM_COUNT; i++) {
		int port = ports[i];
		int server_fd = socket(AF_INET, SOCK_STREAM, 0);
		if (server_fd < 0) {
			LOG_ERROR("TCP video socket create failed for stream %d: %s\n", i, strerror(errno));
			return -1;
		}

		int reuse = 1;
		setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = INADDR_ANY;
		addr.sin_port = htons(port);

		if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			LOG_ERROR("TCP video bind port %d failed: %s\n", port, strerror(errno));
			close(server_fd);
			return -1;
		}

		if (listen(server_fd, 1) < 0) {
			LOG_ERROR("TCP video listen port %d failed: %s\n", port, strerror(errno));
			close(server_fd);
			return -1;
		}

		tcp_video_server_fd[i] = server_fd;
		LOG_INFO("TCP video server listening on port %d for video.%d\n", port, i);
	}

	pthread_create(&tcp_video_thread_0, NULL, tcp_video_accept_thread, (void *)(intptr_t)0);
	pthread_create(&tcp_video_thread_1, NULL, tcp_video_accept_thread, (void *)(intptr_t)1);
	pthread_create(&tcp_video_thread_2, NULL, tcp_video_accept_thread, (void *)(intptr_t)2);
	return 0;
}

static void tcp_video_server_deinit() {
	for (int i = 0; i < TCP_VIDEO_STREAM_COUNT; i++) {
		if (tcp_video_server_fd[i] >= 0) {
			close(tcp_video_server_fd[i]);
			tcp_video_server_fd[i] = -1;
		}
		int fd = __atomic_exchange_n(&tcp_video_client_fd[i], -1, __ATOMIC_SEQ_CST);
		if (fd >= 0)
			close(fd);
	}
	if (tcp_video_thread_0)
		pthread_join(tcp_video_thread_0, NULL);
	if (tcp_video_thread_1)
		pthread_join(tcp_video_thread_1, NULL);
	if (tcp_video_thread_2)
		pthread_join(tcp_video_thread_2, NULL);
}

// ========== TCP Control Server (Port 5102) ==========
static const char *valid_exposure_values[] = {
	"1/50", "1/100", "1/150", "1/200", "1/250",
	"1/500", "1/750", "1/1000", "1/2000", "1/4000"
};
#define NUM_VALID_EXPOSURES (sizeof(valid_exposure_values) / sizeof(valid_exposure_values[0]))

static int is_valid_exposure(const char *value) {
	for (int i = 0; i < (int)NUM_VALID_EXPOSURES; i++) {
		if (strcmp(valid_exposure_values[i], value) == 0)
			return 1;
	}
	return 0;
}

static void tcp_control_handle_command(int client_fd, char *cmd) {
	// Strip trailing \r\n
	char *p = cmd + strlen(cmd) - 1;
	while (p >= cmd && (*p == '\n' || *p == '\r' || *p == ' '))
		*p-- = '\0';

	if (strlen(cmd) == 0)
		return;

	LOG_INFO("TCP control: command=[%s] (len=%d)\n", cmd, (int)strlen(cmd));

	if (strncmp(cmd, "manualExposure:", 15) == 0) {
		const char *exposure_val = cmd + 15;
		if (!is_valid_exposure(exposure_val)) {
			LOG_WARN("TCP control: invalid exposure value '%s'\n", exposure_val);
			const char *resp = "ERR:invalid exposure value\n";
			send(client_fd, resp, strlen(resp), MSG_NOSIGNAL);
			return;
		}
		rk_isp_set_exposure_mode(0, "manual");
		rk_isp_set_exposure_time(0, exposure_val);
		rk_isp_set_exposure_gain(0, 1);
		LOG_INFO("TCP control: manual exposure set to %s, gain=1\n", exposure_val);
		char resp[64];
		snprintf(resp, sizeof(resp), "OK:manualExposure:%s\n", exposure_val);
		send(client_fd, resp, strlen(resp), MSG_NOSIGNAL);
	} else if (strcmp(cmd, "autoExposure") == 0) {
		rk_isp_set_exposure_mode(0, "auto");
		LOG_INFO("TCP control: auto exposure restored\n");
		const char *resp = "OK:autoExposure\n";
		send(client_fd, resp, strlen(resp), MSG_NOSIGNAL);
	} else {
		LOG_WARN("TCP control: unknown command '%s'\n", cmd);
		const char *resp = "ERR:unknown command\n";
		send(client_fd, resp, strlen(resp), MSG_NOSIGNAL);
	}
}

static void *tcp_control_handler_thread(void *arg) {
	int client_fd = (int)(intptr_t)arg;
	prctl(PR_SET_NAME, "TcpCtrlHandler", 0, 0, 0);
	LOG_INFO("TCP control: client connected\n");

	// Set receive timeout so we can check g_video_run_ periodically
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	char buf[256];
	int buf_len = 0;

	while (g_video_run_) {
		int n = recv(client_fd, buf + buf_len, sizeof(buf) - buf_len - 1, 0);
		if (n > 0) {
			buf_len += n;
			buf[buf_len] = '\0';
			LOG_INFO("TCP control: recv %d bytes, buf=[%s] (len=%d)\n", n, buf, buf_len);
			// Process complete lines
			char *newline;
			while ((newline = strchr(buf, '\n')) != NULL) {
				*newline = '\0';
				tcp_control_handle_command(client_fd, buf);
				int consumed = (newline - buf) + 1;
				memmove(buf, newline + 1, buf_len - consumed);
				buf_len -= consumed;
			}
			if (buf_len >= (int)sizeof(buf) - 1) {
				// Buffer overflow, discard
				buf_len = 0;
			}
		} else if (n == 0) {
			// Client disconnected
			LOG_INFO("TCP control: client disconnected, restoring auto exposure\n");
			break;
		} else {
			// recv error: EAGAIN/EWOULDBLOCK means timeout (normal), others mean disconnect
			if (errno != EAGAIN && errno != EWOULDBLOCK)
				break;
		}
	}

	// Restore auto exposure on disconnect
	rk_isp_set_exposure_mode(0, "auto");
	LOG_INFO("TCP control: auto exposure restored on disconnect\n");

	close(client_fd);
	int expected = client_fd;
	__atomic_compare_exchange_n(&tcp_control_client_fd, &expected, -1,
	                            0, __ATOMIC_SEQ_CST, __ATOMIC_RELAXED);
	return NULL;
}

static void *tcp_control_accept_thread(void *arg) {
	prctl(PR_SET_NAME, "TcpCtrlAccept", 0, 0, 0);
	LOG_INFO("TCP control accept thread started on port %d\n", TCP_CONTROL_PORT);

	while (g_video_run_) {
		struct sockaddr_in client_addr;
		socklen_t addr_len = sizeof(client_addr);
		int client_fd = accept(tcp_control_server_fd,
		                       (struct sockaddr *)&client_addr, &addr_len);
		if (client_fd < 0) {
			if (g_video_run_ && errno != EINTR)
				LOG_ERROR("TCP control accept error: %s\n", strerror(errno));
			continue;
		}

		int flag = 1;
		setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

		// Disconnect old client, keep only new one
		int old_fd = __atomic_exchange_n(&tcp_control_client_fd, client_fd, __ATOMIC_SEQ_CST);
		if (old_fd >= 0) {
			LOG_INFO("TCP control: disconnecting old client\n");
			shutdown(old_fd, SHUT_RDWR);
			close(old_fd);
			// Restore auto exposure when old client is kicked
			rk_isp_set_exposure_mode(0, "auto");
		}

		LOG_INFO("TCP control: new client from %s:%d\n",
		         inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

		pthread_t handler;
		pthread_create(&handler, NULL, tcp_control_handler_thread, (void *)(intptr_t)client_fd);
		pthread_detach(handler);
	}
	return NULL;
}

static int tcp_control_server_init(void) {
	int server_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (server_fd < 0) {
		LOG_ERROR("TCP control socket create failed: %s\n", strerror(errno));
		return -1;
	}

	int reuse = 1;
	setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(TCP_CONTROL_PORT);

	if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		LOG_ERROR("TCP control bind port %d failed: %s\n", TCP_CONTROL_PORT, strerror(errno));
		close(server_fd);
		return -1;
	}

	if (listen(server_fd, 1) < 0) {
		LOG_ERROR("TCP control listen port %d failed: %s\n", TCP_CONTROL_PORT, strerror(errno));
		close(server_fd);
		return -1;
	}

	tcp_control_server_fd = server_fd;
	LOG_INFO("TCP control server listening on port %d\n", TCP_CONTROL_PORT);

	pthread_create(&tcp_control_thread, NULL, tcp_control_accept_thread, NULL);
	return 0;
}

static void tcp_control_server_deinit(void) {
	if (tcp_control_server_fd >= 0) {
		close(tcp_control_server_fd);
		tcp_control_server_fd = -1;
	}
	int fd = __atomic_exchange_n(&tcp_control_client_fd, -1, __ATOMIC_SEQ_CST);
	if (fd >= 0) {
		shutdown(fd, SHUT_RDWR);
		close(fd);
	}
	if (tcp_control_thread)
		pthread_join(tcp_control_thread, NULL);
}

static void *rkipc_get_jpeg(void *arg) {
	LOG_DEBUG("#Start %s thread, arg:%p\n", __func__, arg);
	prctl(PR_SET_NAME, "RkipcGetJpeg", 0, 0, 0);
	VENC_STREAM_S stFrame;
	VI_CHN_STATUS_S stChnStatus;
	int loopCount = 0;
	int ret = 0;
	char file_name[128] = {0};
	char record_path[256];

	memset(&record_path, 0, sizeof(record_path));
	strcat(record_path, rk_param_get_string("storage:mount_path", "/userdata"));
	strcat(record_path, "/");
	strcat(record_path, rk_param_get_string("storage.0:folder_name", "video0"));

	stFrame.pstPack = malloc(sizeof(VENC_PACK_S));
	// drop first frame

	ret = RK_MPI_VENC_GetStream(JPEG_VENC_CHN, &stFrame, 1000);
	if (ret == RK_SUCCESS)
		RK_MPI_VENC_ReleaseStream(JPEG_VENC_CHN, &stFrame);
	else
		LOG_ERROR("RK_MPI_VENC_GetStream timeout %x\n", ret);
	while (g_video_run_) {
		if (!get_jpeg_cnt) {
			usleep(300 * 1000);
			continue;
		}
		// 5.get the frame
		ret = RK_MPI_VENC_GetStream(JPEG_VENC_CHN, &stFrame, 1000);
		if (ret == RK_SUCCESS) {
			void *data = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
			LOG_DEBUG("Count:%d, Len:%d, PTS is %" PRId64 ", enH264EType is %d\n", loopCount,
			          stFrame.pstPack->u32Len, stFrame.pstPack->u64PTS,
			          stFrame.pstPack->DataType.enH264EType);
			// save jpeg file
			time_t t = time(NULL);
			struct tm tm = *localtime(&t);
			snprintf(file_name, 128, "%s/%d%02d%02d%02d%02d%02d.jpeg", record_path,
			         tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
			         tm.tm_sec);
			LOG_DEBUG("file_name is %s, u32Len is %d\n", file_name, stFrame.pstPack->u32Len);
			FILE *fp = fopen(file_name, "wb");
			if (fp == NULL) {
				LOG_ERROR("fp is NULL\n");
			} else {
				fwrite(data, 1, stFrame.pstPack->u32Len, fp);
			}
			// 7.release the frame
			ret = RK_MPI_VENC_ReleaseStream(JPEG_VENC_CHN, &stFrame);
			if (ret != RK_SUCCESS) {
				LOG_ERROR("RK_MPI_VENC_ReleaseStream fail %x\n", ret);
			}
			if (fp) {
				fflush(fp);
				fclose(fp);
			}
			loopCount++;
		} else {
			LOG_ERROR("RK_MPI_VENC_GetStream timeout %x\n", ret);
		}
		get_jpeg_cnt--;
		RK_MPI_VENC_StopRecvFrame(JPEG_VENC_CHN);
		// usleep(33 * 1000);
	}
	if (stFrame.pstPack)
		free(stFrame.pstPack);

	return 0;
}

static void *rkipc_cycle_snapshot(void *arg) {
	LOG_INFO("start %s thread, arg:%p\n", __func__, arg);
	prctl(PR_SET_NAME, "RkipcCycleSnapshot", 0, 0, 0);

	while (g_video_run_ && cycle_snapshot_flag) {
		usleep(rk_param_get_int("video.jpeg:snapshot_interval_ms", 1000) * 1000);
		rk_take_photo();
	}
	LOG_INFO("exit %s thread, arg:%p\n", __func__, arg);

	return 0;
}

static void *rkipc_get_vi_2_send(void *arg) {
	LOG_DEBUG("#Start %s thread, arg:%p\n", __func__, arg);
	prctl(PR_SET_NAME, "RkipcGetVi2", 0, 0, 0);
	int ret;
	int32_t loopCount = 0;
	VIDEO_FRAME_INFO_S stViFrame;
	int npu_cycle_time_ms = 1000 / rk_param_get_int("video.source:npu_fps", 10);

	long long before_time, cost_time;
	while (g_video_run_) {
		before_time = rkipc_get_curren_time_ms();
		ret = RK_MPI_VI_GetChnFrame(pipe_id_, VIDEO_PIPE_2, &stViFrame, 1000);
		if (ret == RK_SUCCESS) {
			void *data = RK_MPI_MB_Handle2VirAddr(stViFrame.stVFrame.pMbBlk);
			uint8_t *phy_addr = (uint8_t *)RK_MPI_MB_Handle2PhysAddr(stViFrame.stVFrame.pMbBlk);
			rkipc_rockiva_write_nv12_frame_by_phy_addr(
			    stViFrame.stVFrame.u32Width, stViFrame.stVFrame.u32Height, loopCount, phy_addr);
			ret = RK_MPI_VI_ReleaseChnFrame(pipe_id_, VIDEO_PIPE_2, &stViFrame);
			if (ret != RK_SUCCESS)
				LOG_ERROR("RK_MPI_VI_ReleaseChnFrame fail %x", ret);
			loopCount++;
		} else {
			LOG_ERROR("RK_MPI_VI_GetChnFrame timeout %x", ret);
		}
		cost_time = rkipc_get_curren_time_ms() - before_time;
		if ((cost_time > 0) && (cost_time < npu_cycle_time_ms))
			usleep((npu_cycle_time_ms - cost_time) * 1000);
	}
	return NULL;
}

static void *rkipc_get_vpss_bgr(void *arg) {
	LOG_DEBUG("#Start %s thread, arg:%p\n", __func__, arg);
	prctl(PR_SET_NAME, "RkipcGetVpssBgr", 0, 0, 0);
	VIDEO_FRAME_INFO_S frame;
	VI_CHN_STATUS_S stChnStatus;
	int32_t loopCount = 0;
	int ret = 0;

	while (g_video_run_) {
		ret = RK_MPI_VPSS_GetChnFrame(VPSS_BGR, 0, &frame, 1000);
		if (ret == RK_SUCCESS) {
			void *data = RK_MPI_MB_Handle2VirAddr(frame.stVFrame.pMbBlk);
			// LOG_INFO("data:%p, u32Width:%d, u32Height:%d, PTS is %" PRId64 "\n", data,
			//          frame.stVFrame.u32Width, frame.stVFrame.u32Height, frame.stVFrame.u64PTS);
			// rkipc_rockiva_write_rgb888_frame(frame.stVFrame.u32Width, frame.stVFrame.u32Height,
			//                                  data);
			int32_t fd = RK_MPI_MB_Handle2Fd(frame.stVFrame.pMbBlk);
#if 0
			FILE *fp = fopen("/data/test.bgr", "wb");
			fwrite(data, 1, frame.stVFrame.u32Width * frame.stVFrame.u32Height * 3, fp);
			fflush(fp);
			fclose(fp);
			exit(1);
#endif
			// long long last_nn_time = rkipc_get_curren_time_ms();
			rkipc_rockiva_write_rgb888_frame_by_fd(frame.stVFrame.u32Width,
			                                       frame.stVFrame.u32Height, loopCount, fd);
			// LOG_DEBUG("nn time-consuming is %lld\n",(rkipc_get_curren_time_ms() - last_nn_time));

			ret = RK_MPI_VPSS_ReleaseChnFrame(VPSS_BGR, 0, &frame);
			if (ret != RK_SUCCESS)
				LOG_ERROR("RK_MPI_VPSS_ReleaseChnFrame fail %x\n", ret);
			loopCount++;
		} else {
			LOG_ERROR("RK_MPI_VPSS_GetChnFrame timeout %x\n", ret);
		}
	}

	return 0;
}

static void *rkipc_ivs_get_results(void *arg) {
	LOG_DEBUG("#Start %s thread, arg:%p\n", __func__, arg);
	prctl(PR_SET_NAME, "RkipcGetIVS", 0, 0, 0);
	int ret, i;
	IVS_RESULT_INFO_S stResults;
	int resultscount = 0;
	int count = 0;
	int md = rk_param_get_int("ivs:md", 0);
	int od = rk_param_get_int("ivs:od", 0);
	int width = rk_param_get_int("video.2:width", 960);
	int height = rk_param_get_int("video.2:height", 540);
	int md_area_threshold = width * height * 0.3;

	while (g_video_run_) {
		ret = RK_MPI_IVS_GetResults(0, &stResults, 1000);
		if (ret >= 0) {
			resultscount++;
			if (md == 1) {
				if (stResults.pstResults->stMdInfo.u32Square > md_area_threshold) {
					LOG_INFO("MD: md_area is %d, md_area_threshold is %d\n",
					         stResults.pstResults->stMdInfo.u32Square, md_area_threshold);
				}
			}
			if (od == 1) {
				if (stResults.s32ResultNum > 0) {
					if (stResults.pstResults->stOdInfo.u32Flag)
						LOG_INFO("OD flag:%d\n", stResults.pstResults->stOdInfo.u32Flag);
				}
			}
			RK_MPI_IVS_ReleaseResults(0, &stResults);
		} else {
			LOG_ERROR("get chn %d fail %d\n", 0, ret);
			usleep(50000llu);
		}
	}
	return NULL;
}

int rkipc_rtmp_init() {
	int ret = 0;
	ret |= rk_rtmp_init(0, RTMP_URL_0);
	ret |= rk_rtmp_init(1, RTMP_URL_1);
	// ret |= rk_rtmp_init(2, RTMP_URL_2);

	return ret;
}

int rkipc_rtmp_deinit() {
	int ret = 0;
	ret |= rk_rtmp_deinit(0);
	ret |= rk_rtmp_deinit(1);
	// ret |= rk_rtmp_deinit(2);

	return ret;
}

int rkipc_vi_dev_init() {
	LOG_DEBUG("%s\n", __func__);
	int ret = 0;
	VI_DEV_ATTR_S stDevAttr;
	VI_DEV_BIND_PIPE_S stBindPipe;
	memset(&stDevAttr, 0, sizeof(stDevAttr));
	memset(&stBindPipe, 0, sizeof(stBindPipe));
	// 0. get dev config status
	ret = RK_MPI_VI_GetDevAttr(dev_id_, &stDevAttr);
	if (ret == RK_ERR_VI_NOT_CONFIG) {
		// 0-1.config dev
		ret = RK_MPI_VI_SetDevAttr(dev_id_, &stDevAttr);
		if (ret != RK_SUCCESS) {
			LOG_ERROR("RK_MPI_VI_SetDevAttr %x\n", ret);
			return -1;
		}
	} else {
		LOG_ERROR("RK_MPI_VI_SetDevAttr already\n");
	}
	// 1.get dev enable status
	ret = RK_MPI_VI_GetDevIsEnable(dev_id_);
	if (ret != RK_SUCCESS) {
		// 1-2.enable dev
		ret = RK_MPI_VI_EnableDev(dev_id_);
		if (ret != RK_SUCCESS) {
			LOG_ERROR("RK_MPI_VI_EnableDev %x\n", ret);
			return -1;
		}
		// 1-3.bind dev/pipe
		stBindPipe.u32Num = pipe_id_;
		stBindPipe.PipeId[0] = pipe_id_;
		ret = RK_MPI_VI_SetDevBindPipe(dev_id_, &stBindPipe);
		if (ret != RK_SUCCESS) {
			LOG_ERROR("RK_MPI_VI_SetDevBindPipe %x\n", ret);
			return -1;
		}
	} else {
		LOG_ERROR("RK_MPI_VI_EnableDev already\n");
	}

	return 0;
}

int rkipc_vi_dev_deinit() {
	RK_MPI_VI_DisableDev(pipe_id_);

	return 0;
}

int rkipc_pipe_0_init() {
	int ret;
	int video_width = rk_param_get_int("video.0:width", -1);
	int video_height = rk_param_get_int("video.0:height", -1);
	int video_max_width = rk_param_get_int("video.0:max_width", -1);
	int video_max_height = rk_param_get_int("video.0:max_height", -1);
	int buffer_line = rk_param_get_int("video.source:buffer_line", video_max_height / 4);
	if (buffer_line < 128)
		buffer_line = video_max_height;
	int rotation = rk_param_get_int("video.source:rotation", 0);
	int buf_cnt = 2;
	int frame_min_i_qp = rk_param_get_int("video.0:frame_min_i_qp", 26);
	int frame_min_qp = rk_param_get_int("video.0:frame_min_qp", 28);
	int frame_max_i_qp = rk_param_get_int("video.0:frame_max_i_qp", 51);
	int frame_max_qp = rk_param_get_int("video.0:frame_max_qp", 51);
	int scalinglist = rk_param_get_int("video.0:scalinglist", 0);

	// VI
	VI_CHN_ATTR_S vi_chn_attr;
	memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
	vi_chn_attr.stIspOpt.u32BufCount = buf_cnt;
	vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	vi_chn_attr.stIspOpt.stMaxSize.u32Width = rk_param_get_int("video.0:max_width", 2560);
	vi_chn_attr.stIspOpt.stMaxSize.u32Height = rk_param_get_int("video.0:max_height", 1440);
	vi_chn_attr.stSize.u32Width = video_width;
	vi_chn_attr.stSize.u32Height = video_height;
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.u32Depth = 0;
	vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	ret = RK_MPI_VI_SetChnAttr(pipe_id_, VIDEO_PIPE_0, &vi_chn_attr);
	if (ret) {
		LOG_ERROR("ERROR: create VI error! ret=%d\n", ret);
		return ret;
	}

	VI_CHN_BUF_WRAP_S stViWrap;
	memset(&stViWrap, 0, sizeof(VI_CHN_BUF_WRAP_S));
	if (enable_wrap) {
		if (buffer_line < 128 || buffer_line > video_max_height) {
			LOG_ERROR("wrap mode buffer line must between [128, H], set as video_max_height\n");
			buffer_line = video_max_height;
		}
		stViWrap.bEnable = enable_wrap;
		stViWrap.u32BufLine = buffer_line;
		stViWrap.u32WrapBufferSize = stViWrap.u32BufLine * video_max_width * 3 / 2;
		LOG_INFO("set vi channel wrap line: %d, wrapBuffSize = %d\n", stViWrap.u32BufLine,
		         stViWrap.u32WrapBufferSize);
		RK_MPI_VI_SetChnWrapBufAttr(pipe_id_, VIDEO_PIPE_0, &stViWrap);
	}

	ret = RK_MPI_VI_EnableChn(pipe_id_, VIDEO_PIPE_0);
	if (ret) {
		LOG_ERROR("ERROR: create VI error! ret=%d\n", ret);
		return ret;
	}

	// VENC
	VENC_CHN_ATTR_S venc_chn_attr;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	tmp_output_data_type = rk_param_get_string("video.0:output_data_type", NULL);
	tmp_rc_mode = rk_param_get_string("video.0:rc_mode", NULL);
	tmp_h264_profile = rk_param_get_string("video.0:h264_profile", NULL);
	if ((tmp_output_data_type == NULL) || (tmp_rc_mode == NULL)) {
		LOG_ERROR("tmp_output_data_type or tmp_rc_mode is NULL\n");
		return -1;
	}
	LOG_DEBUG("tmp_output_data_type is %s, tmp_rc_mode is %s, tmp_h264_profile is %s\n",
	          tmp_output_data_type, tmp_rc_mode, tmp_h264_profile);
	if (!strcmp(tmp_output_data_type, "H.264")) {
		venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_AVC;
		if (!strcmp(tmp_h264_profile, "high"))
			venc_chn_attr.stVencAttr.u32Profile = 100;
		else if (!strcmp(tmp_h264_profile, "main"))
			venc_chn_attr.stVencAttr.u32Profile = 77;
		else if (!strcmp(tmp_h264_profile, "baseline"))
			venc_chn_attr.stVencAttr.u32Profile = 66;
		else
			LOG_ERROR("tmp_h264_profile is %s\n", tmp_h264_profile);
		if (!strcmp(tmp_rc_mode, "CBR")) {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
			venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = rk_param_get_int("video.0:gop", -1);
			venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = rk_param_get_int("video.0:max_rate", -1);
		} else {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
			venc_chn_attr.stRcAttr.stH264Vbr.u32Gop = rk_param_get_int("video.0:gop", -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32BitRate = rk_param_get_int("video.0:mid_rate", -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32MaxBitRate =
			    rk_param_get_int("video.0:max_rate", -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32MinBitRate =
			    rk_param_get_int("video.0:min_rate", -1);
		}
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_HEVC;
		if (!strcmp(tmp_rc_mode, "CBR")) {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
			venc_chn_attr.stRcAttr.stH265Cbr.u32Gop = rk_param_get_int("video.0:gop", -1);
			venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = rk_param_get_int("video.0:max_rate", -1);
		} else {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
			venc_chn_attr.stRcAttr.stH265Vbr.u32Gop = rk_param_get_int("video.0:gop", -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32BitRate = rk_param_get_int("video.0:mid_rate", -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32MaxBitRate =
			    rk_param_get_int("video.0:max_rate", -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32MinBitRate =
			    rk_param_get_int("video.0:min_rate", -1);
		}
	} else {
		LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
		return -1;
	}
	tmp_smart = rk_param_get_string("video.0:smart", NULL);
	tmp_gop_mode = rk_param_get_string("video.0:gop_mode", NULL);
	if (!strcmp(tmp_gop_mode, "normalP")) {
		venc_chn_attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
	} else if (!strcmp(tmp_gop_mode, "smartP")) {
		venc_chn_attr.stGopAttr.enGopMode = VENC_GOPMODE_SMARTP;
		venc_chn_attr.stGopAttr.s32VirIdrLen = rk_param_get_int("video.0:smartp_viridrlen", 25);
		venc_chn_attr.stGopAttr.u32MaxLtrCount = 1; // long-term reference frame ltr is fixed to 1
	} else if (!strcmp(tmp_gop_mode, "tsvc4")) {
		venc_chn_attr.stGopAttr.enGopMode = VENC_GOPMODE_TSVC4;
	}
	// venc_chn_attr.stGopAttr.u32GopSize = rk_param_get_int("video.0:gop", -1);
	venc_chn_attr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	venc_chn_attr.stVencAttr.u32MaxPicWidth = rk_param_get_int("video.0:max_width", 2560);
	venc_chn_attr.stVencAttr.u32MaxPicHeight = rk_param_get_int("video.0:max_height", 1440);
	venc_chn_attr.stVencAttr.u32PicWidth = video_width;
	venc_chn_attr.stVencAttr.u32PicHeight = video_height;
	venc_chn_attr.stVencAttr.u32VirWidth = video_width;
	venc_chn_attr.stVencAttr.u32VirHeight = video_height;
	venc_chn_attr.stVencAttr.u32StreamBufCnt = rk_param_get_int("video.0:buffer_count", 4);
	venc_chn_attr.stVencAttr.u32BufSize = rk_param_get_int("video.0:buffer_size", 1843200);
	// venc_chn_attr.stVencAttr.u32Depth = 1;
	ret = RK_MPI_VENC_CreateChn(VIDEO_PIPE_0, &venc_chn_attr);
	if (ret) {
		LOG_ERROR("ERROR: create VENC error! ret=%#x\n", ret);
		return -1;
	}
	rk_video_reset_frame_rate(VIDEO_PIPE_0);

	if (!strcmp(tmp_smart, "open"))
		RK_MPI_VENC_EnableSvc(VIDEO_PIPE_0, 1);

	if (rk_param_get_int("video.0:enable_motion_deblur", 0)) {
		ret = RK_MPI_VENC_EnableMotionDeblur(VIDEO_PIPE_0, true);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_EnableMotionDeblur error! ret=%#x\n", ret);
	}
	if (rk_param_get_int("video.0:enable_motion_static_switch", 0)) {
		ret = RK_MPI_VENC_EnableMotionStaticSwitch(VIDEO_PIPE_0, true);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_EnableMotionStaticSwitch error! ret=%#x\n", ret);
	}

	VENC_RC_PARAM2_S rc_param2;
	ret = RK_MPI_VENC_GetRcParam2(VIDEO_PIPE_0, &rc_param2);
	if (ret)
		LOG_ERROR("RK_MPI_VENC_GetRcParam2 error! ret=%#x\n", ret);
	const char *strings = rk_param_get_string("video.0:thrd_i", NULL);
	if (strings) {
		char *str = strdup(strings);
		if (str) {
			char *tmp = str;
			char *token = strsep(&tmp, ",");
			int i = 0;
			while (token != NULL) {
				rc_param2.u32ThrdI[i++] = atoi(token);
				token = strsep(&tmp, ",");
			}
			free(str);
		}
	}
	strings = rk_param_get_string("video.0:thrd_p", NULL);
	if (strings) {
		char *str = strdup(strings);
		if (str) {
			char *tmp = str;
			char *token = strsep(&tmp, ",");
			int i = 0;
			while (token != NULL) {
				rc_param2.u32ThrdP[i++] = atoi(token);
				token = strsep(&tmp, ",");
			}
			free(str);
		}
	}
	strings = rk_param_get_string("video.0:aq_step_i", NULL);
	if (strings) {
		char *str = strdup(strings);
		if (str) {
			char *tmp = str;
			char *token = strsep(&tmp, ",");
			int i = 0;
			while (token != NULL) {
				rc_param2.s32AqStepI[i++] = atoi(token);
				token = strsep(&tmp, ",");
			}
			free(str);
		}
	}
	strings = rk_param_get_string("video.0:aq_step_p", NULL);
	if (strings) {
		char *str = strdup(strings);
		if (str) {
			char *tmp = str;
			char *token = strsep(&tmp, ",");
			int i = 0;
			while (token != NULL) {
				rc_param2.s32AqStepP[i++] = atoi(token);
				token = strsep(&tmp, ",");
			}
			free(str);
		}
	}
	ret = RK_MPI_VENC_SetRcParam2(VIDEO_PIPE_0, &rc_param2);
	if (ret)
		LOG_ERROR("RK_MPI_VENC_SetRcParam2 error! ret=%#x\n", ret);

	if (!strcmp(tmp_output_data_type, "H.264")) {
		VENC_H264_QBIAS_S qbias;
		qbias.bEnable = rk_param_get_int("video.0:qbias_enable", 0);
		qbias.u32QbiasI = rk_param_get_int("video.0:qbias_i", 0);
		qbias.u32QbiasP = rk_param_get_int("video.0:qbias_p", 0);
		ret = RK_MPI_VENC_SetH264Qbias(VIDEO_PIPE_0, &qbias);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_SetH264Qbias error! ret=%#x\n", ret);
	} else {
		VENC_H265_QBIAS_S qbias;
		qbias.bEnable = rk_param_get_int("video.0:qbias_enable", 0);
		qbias.u32QbiasI = rk_param_get_int("video.0:qbias_i", 0);
		qbias.u32QbiasP = rk_param_get_int("video.0:qbias_p", 0);
		ret = RK_MPI_VENC_SetH265Qbias(VIDEO_PIPE_0, &qbias);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_SetH265Qbias error! ret=%#x\n", ret);
	}

	VENC_FILTER_S pstFilter;
	RK_MPI_VENC_GetFilter(VIDEO_PIPE_0, &pstFilter);
	pstFilter.u32StrengthI = rk_param_get_int("video.0:flt_str_i", 0);
	pstFilter.u32StrengthP = rk_param_get_int("video.0:flt_str_p", 0);
	RK_MPI_VENC_SetFilter(VIDEO_PIPE_0, &pstFilter);

	// VENC_RC_PARAM_S h265_RcParam;
	// RK_MPI_VENC_GetRcParam(VIDEO_PIPE_0, &h265_RcParam);
	// h265_RcParam.s32FirstFrameStartQp = 26;
	// h265_RcParam.stParamH265.u32StepQp = 8;
	// h265_RcParam.stParamH265.u32MaxQp = 51;
	// h265_RcParam.stParamH265.u32MinQp = 10;
	// h265_RcParam.stParamH265.u32MaxIQp = 46;
	// h265_RcParam.stParamH265.u32MinIQp = 24;
	// h265_RcParam.stParamH265.s32DeltIpQp = -4;
	// RK_MPI_VENC_SetRcParam(VIDEO_PIPE_0, &h265_RcParam);

	tmp_rc_quality = rk_param_get_string("video.0:rc_quality", NULL);
	VENC_RC_PARAM_S venc_rc_param;
	RK_MPI_VENC_GetRcParam(VIDEO_PIPE_0, &venc_rc_param);
	if (!strcmp(tmp_output_data_type, "H.264")) {
		if (!strcmp(tmp_rc_quality, "highest")) {
			venc_rc_param.stParamH264.u32MinQp = 10;
		} else if (!strcmp(tmp_rc_quality, "higher")) {
			venc_rc_param.stParamH264.u32MinQp = 15;
		} else if (!strcmp(tmp_rc_quality, "high")) {
			venc_rc_param.stParamH264.u32MinQp = 20;
		} else if (!strcmp(tmp_rc_quality, "medium")) {
			venc_rc_param.stParamH264.u32MinQp = 25;
		} else if (!strcmp(tmp_rc_quality, "low")) {
			venc_rc_param.stParamH264.u32MinQp = 30;
		} else if (!strcmp(tmp_rc_quality, "lower")) {
			venc_rc_param.stParamH264.u32MinQp = 35;
		} else {
			venc_rc_param.stParamH264.u32MinQp = 40;
		}
		venc_rc_param.stParamH264.u32FrmMinIQp = frame_min_i_qp;
		venc_rc_param.stParamH264.u32FrmMinQp = frame_min_qp;
		venc_rc_param.stParamH264.u32FrmMaxIQp = frame_max_i_qp;
		venc_rc_param.stParamH264.u32FrmMaxQp = frame_max_qp;
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		if (!strcmp(tmp_rc_quality, "highest")) {
			venc_rc_param.stParamH265.u32MinQp = 10;
		} else if (!strcmp(tmp_rc_quality, "higher")) {
			venc_rc_param.stParamH265.u32MinQp = 15;
		} else if (!strcmp(tmp_rc_quality, "high")) {
			venc_rc_param.stParamH265.u32MinQp = 20;
		} else if (!strcmp(tmp_rc_quality, "medium")) {
			venc_rc_param.stParamH265.u32MinQp = 25;
		} else if (!strcmp(tmp_rc_quality, "low")) {
			venc_rc_param.stParamH265.u32MinQp = 30;
		} else if (!strcmp(tmp_rc_quality, "lower")) {
			venc_rc_param.stParamH265.u32MinQp = 35;
		} else {
			venc_rc_param.stParamH265.u32MinQp = 40;
		}
		venc_rc_param.stParamH265.u32FrmMinIQp = frame_min_i_qp;
		venc_rc_param.stParamH265.u32FrmMinQp = frame_min_qp;
		venc_rc_param.stParamH265.u32FrmMaxIQp = frame_max_i_qp;
		venc_rc_param.stParamH265.u32FrmMaxQp = frame_max_qp;
	} else {
		LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
		return -1;
	}
	RK_MPI_VENC_SetRcParam(VIDEO_PIPE_0, &venc_rc_param);

	VENC_CHN_BUF_WRAP_S stVencChnBufWrap;
	memset(&stVencChnBufWrap, 0, sizeof(stVencChnBufWrap));
	if (enable_wrap) {
		stVencChnBufWrap.bEnable = enable_wrap;
		stVencChnBufWrap.u32BufLine =
		    rk_param_get_int("video.source:buffer_line", video_max_height);
		if (stVencChnBufWrap.u32BufLine < 128)
			stVencChnBufWrap.u32BufLine = video_max_height;
		RK_MPI_VENC_SetChnBufWrapAttr(VIDEO_PIPE_0, &stVencChnBufWrap);
	}

	if (!strcmp(tmp_output_data_type, "H.264")) {
		VENC_H264_TRANS_S pstH264Trans;
		RK_MPI_VENC_GetH264Trans(VIDEO_PIPE_0, &pstH264Trans);
		pstH264Trans.bScalingListValid = scalinglist;
		RK_MPI_VENC_SetH264Trans(VIDEO_PIPE_0, &pstH264Trans);
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		VENC_H265_TRANS_S pstH265Trans;
		RK_MPI_VENC_GetH265Trans(VIDEO_PIPE_0, &pstH265Trans);
		pstH265Trans.bScalingListEnabled = scalinglist;
		RK_MPI_VENC_SetH265Trans(VIDEO_PIPE_0, &pstH265Trans);
	}
	VENC_CHN_REF_BUF_SHARE_S stVencChnRefBufShare;
	memset(&stVencChnRefBufShare, 0, sizeof(VENC_CHN_REF_BUF_SHARE_S));
	stVencChnRefBufShare.bEnable = rk_param_get_int("video.0:enable_refer_buffer_share", 0);
	RK_MPI_VENC_SetChnRefBufShareAttr(VIDEO_PIPE_0, &stVencChnRefBufShare);
	if (rotation == 0) {
		RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_0, ROTATION_0);
	} else if (rotation == 90) {
		RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_0, ROTATION_90);
	} else if (rotation == 180) {
		RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_0, ROTATION_180);
	} else if (rotation == 270) {
		RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_0, ROTATION_270);
	}

	VENC_RECV_PIC_PARAM_S stRecvParam;
	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(VIDEO_PIPE_0, &stRecvParam);
	pthread_create(&venc_thread_0, NULL, rkipc_get_venc_lazy, (void *)(intptr_t)0);
	// NOTE: VI->VENC bind moved to rkipc_get_venc_lazy (lazy binding)

	return 0;
}

int rkipc_pipe_0_deinit() {
	int ret;
	// unbind only if still bound (lazy binding)
	if (pipe_0_bound) {
		MPP_CHN_S lvi, lvenc;
		lvi.enModId = RK_ID_VI;
		lvi.s32DevId = 0;
		lvi.s32ChnId = VIDEO_PIPE_0;
		lvenc.enModId = RK_ID_VENC;
		lvenc.s32DevId = 0;
		lvenc.s32ChnId = VIDEO_PIPE_0;
		ret = RK_MPI_SYS_UnBind(&lvi, &lvenc);
		if (ret)
			LOG_ERROR("Unbind VI and VENC error! ret=%#x\n", ret);
		pipe_0_bound = 0;
	}
	// VENC
	ret = RK_MPI_VENC_StopRecvFrame(VIDEO_PIPE_0);
	ret |= RK_MPI_VENC_DestroyChn(VIDEO_PIPE_0);
	if (ret)
		LOG_ERROR("ERROR: Destroy VENC error! ret=%#x\n", ret);
	else
		LOG_DEBUG("RK_MPI_VENC_DestroyChn success\n");
	// VI
	ret = RK_MPI_VI_DisableChn(pipe_id_, VIDEO_PIPE_0);
	if (ret)
		LOG_ERROR("ERROR: Destroy VI error! ret=%#x\n", ret);

	return 0;
}

int rkipc_pipe_1_init() {
	int ret;
	int video_width = rk_param_get_int("video.1:width", 1920);
	int video_height = rk_param_get_int("video.1:height", 1080);
	int buf_cnt = rk_param_get_int("video.1:input_buffer_count", 2);
	int rotation = rk_param_get_int("video.source:rotation", 0);
	int frame_min_i_qp = rk_param_get_int("video.1:frame_min_i_qp", 26);
	int frame_min_qp = rk_param_get_int("video.1:frame_min_qp", 28);
	int frame_max_i_qp = rk_param_get_int("video.1:frame_max_i_qp", 51);
	int frame_max_qp = rk_param_get_int("video.1:frame_max_qp", 51);
	int scalinglist = rk_param_get_int("video.1:scalinglist", 0);

	// VI
	VI_CHN_ATTR_S vi_chn_attr;
	memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
	vi_chn_attr.stIspOpt.u32BufCount = buf_cnt;
	vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	vi_chn_attr.stIspOpt.stMaxSize.u32Width = rk_param_get_int("video.1:max_width", 704);
	vi_chn_attr.stIspOpt.stMaxSize.u32Height = rk_param_get_int("video.1:max_height", 576);
	vi_chn_attr.stSize.u32Width = video_width;
	vi_chn_attr.stSize.u32Height = video_height;
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.u32Depth = 0;
	if (g_enable_vo)
		vi_chn_attr.u32Depth += 1;
	ret = RK_MPI_VI_SetChnAttr(pipe_id_, VIDEO_PIPE_1, &vi_chn_attr);
	ret |= RK_MPI_VI_EnableChn(pipe_id_, VIDEO_PIPE_1);
	if (ret) {
		LOG_ERROR("ERROR: create VI error! ret=%d\n", ret);
		return ret;
	}

	// VENC
	VENC_CHN_ATTR_S venc_chn_attr;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	tmp_output_data_type = rk_param_get_string("video.1:output_data_type", NULL);
	tmp_rc_mode = rk_param_get_string("video.1:rc_mode", NULL);
	tmp_h264_profile = rk_param_get_string("video.1:h264_profile", NULL);
	if ((tmp_output_data_type == NULL) || (tmp_rc_mode == NULL)) {
		LOG_ERROR("tmp_output_data_type or tmp_rc_mode is NULL\n");
		return -1;
	}
	LOG_DEBUG("tmp_output_data_type is %s, tmp_rc_mode is %s, tmp_h264_profile is %s\n",
	          tmp_output_data_type, tmp_rc_mode, tmp_h264_profile);
	if (!strcmp(tmp_output_data_type, "H.264")) {
		venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_AVC;
		if (!strcmp(tmp_h264_profile, "high"))
			venc_chn_attr.stVencAttr.u32Profile = 100;
		else if (!strcmp(tmp_h264_profile, "main"))
			venc_chn_attr.stVencAttr.u32Profile = 77;
		else if (!strcmp(tmp_h264_profile, "baseline"))
			venc_chn_attr.stVencAttr.u32Profile = 66;
		else
			LOG_ERROR("tmp_h264_profile is %s\n", tmp_h264_profile);
		if (!strcmp(tmp_rc_mode, "CBR")) {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
			venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = rk_param_get_int("video.1:gop", -1);
			venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = rk_param_get_int("video.1:max_rate", -1);
		} else {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
			venc_chn_attr.stRcAttr.stH264Vbr.u32Gop = rk_param_get_int("video.1:gop", -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32BitRate = rk_param_get_int("video.1:mid_rate", -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32MaxBitRate =
			    rk_param_get_int("video.1:max_rate", -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32MinBitRate =
			    rk_param_get_int("video.1:min_rate", -1);
		}
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_HEVC;
		if (!strcmp(tmp_rc_mode, "CBR")) {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
			venc_chn_attr.stRcAttr.stH265Cbr.u32Gop = rk_param_get_int("video.1:gop", -1);
			venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = rk_param_get_int("video.1:max_rate", -1);
		} else {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
			venc_chn_attr.stRcAttr.stH265Vbr.u32Gop = rk_param_get_int("video.1:gop", -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32BitRate = rk_param_get_int("video.1:mid_rate", -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32MaxBitRate =
			    rk_param_get_int("video.1:max_rate", -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32MinBitRate =
			    rk_param_get_int("video.1:min_rate", -1);
		}
	} else {
		LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
		return -1;
	}
	tmp_smart = rk_param_get_string("video.1:smart", NULL);
	tmp_gop_mode = rk_param_get_string("video.1:gop_mode", NULL);
	if (!strcmp(tmp_gop_mode, "normalP")) {
		venc_chn_attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
	} else if (!strcmp(tmp_gop_mode, "smartP")) {
		venc_chn_attr.stGopAttr.enGopMode = VENC_GOPMODE_SMARTP;
		venc_chn_attr.stGopAttr.s32VirIdrLen = rk_param_get_int("video.1:smartp_viridrlen", 25);
		venc_chn_attr.stGopAttr.u32MaxLtrCount = 1; // long-term reference frame ltr is fixed to 1
	} else if (!strcmp(tmp_gop_mode, "tsvc4")) {
		venc_chn_attr.stGopAttr.enGopMode = VENC_GOPMODE_TSVC4;
	}
	// venc_chn_attr.stGopAttr.u32GopSize = rk_param_get_int("video.1:gop", -1);

	venc_chn_attr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	venc_chn_attr.stVencAttr.u32MaxPicWidth = rk_param_get_int("video.1:max_width", 704);
	venc_chn_attr.stVencAttr.u32MaxPicHeight = rk_param_get_int("video.1:max_height", 576);
	venc_chn_attr.stVencAttr.u32PicWidth = video_width;
	venc_chn_attr.stVencAttr.u32PicHeight = video_height;
	venc_chn_attr.stVencAttr.u32VirWidth = video_width;
	venc_chn_attr.stVencAttr.u32VirHeight = video_height;
	venc_chn_attr.stVencAttr.u32StreamBufCnt = rk_param_get_int("video.1:buffer_count", 4);
	venc_chn_attr.stVencAttr.u32BufSize = rk_param_get_int("video.1:buffer_size", 202752);
	// venc_chn_attr.stVencAttr.u32Depth = 1;
	ret = RK_MPI_VENC_CreateChn(VIDEO_PIPE_1, &venc_chn_attr);
	if (ret) {
		LOG_ERROR("ERROR: create VENC error! ret=%#x\n", ret);
		return -1;
	}
	rk_video_reset_frame_rate(VIDEO_PIPE_1);

	if (!strcmp(tmp_smart, "open"))
		RK_MPI_VENC_EnableSvc(VIDEO_PIPE_1, 1);

	if (rk_param_get_int("video.1:enable_motion_deblur", 0)) {
		ret = RK_MPI_VENC_EnableMotionDeblur(VIDEO_PIPE_1, true);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_EnableMotionDeblur error! ret=%#x\n", ret);
	}
	if (rk_param_get_int("video.1:enable_motion_static_switch", 0)) {
		ret = RK_MPI_VENC_EnableMotionStaticSwitch(VIDEO_PIPE_1, true);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_EnableMotionStaticSwitch error! ret=%#x\n", ret);
	}

	VENC_RC_PARAM2_S rc_param2;
	ret = RK_MPI_VENC_GetRcParam2(VIDEO_PIPE_1, &rc_param2);
	if (ret)
		LOG_ERROR("RK_MPI_VENC_GetRcParam2 error! ret=%#x\n", ret);
	const char *strings = rk_param_get_string("video.1:thrd_i", NULL);
	if (strings) {
		char *str = strdup(strings);
		if (str) {
			char *tmp = str;
			char *token = strsep(&tmp, ",");
			int i = 0;
			while (token != NULL) {
				rc_param2.u32ThrdI[i++] = atoi(token);
				token = strsep(&tmp, ",");
			}
			free(str);
		}
	}
	strings = rk_param_get_string("video.1:thrd_p", NULL);
	if (strings) {
		char *str = strdup(strings);
		if (str) {
			char *tmp = str;
			char *token = strsep(&tmp, ",");
			int i = 0;
			while (token != NULL) {
				rc_param2.u32ThrdP[i++] = atoi(token);
				token = strsep(&tmp, ",");
			}
			free(str);
		}
	}
	strings = rk_param_get_string("video.1:aq_step_i", NULL);
	if (strings) {
		char *str = strdup(strings);
		if (str) {
			char *tmp = str;
			char *token = strsep(&tmp, ",");
			int i = 0;
			while (token != NULL) {
				rc_param2.s32AqStepI[i++] = atoi(token);
				token = strsep(&tmp, ",");
			}
			free(str);
		}
	}
	strings = rk_param_get_string("video.1:aq_step_p", NULL);
	if (strings) {
		char *str = strdup(strings);
		if (str) {
			char *tmp = str;
			char *token = strsep(&tmp, ",");
			int i = 0;
			while (token != NULL) {
				rc_param2.s32AqStepP[i++] = atoi(token);
				token = strsep(&tmp, ",");
			}
			free(str);
		}
	}
	ret = RK_MPI_VENC_SetRcParam2(VIDEO_PIPE_1, &rc_param2);
	if (ret)
		LOG_ERROR("RK_MPI_VENC_SetRcParam2 error! ret=%#x\n", ret);

	if (!strcmp(tmp_output_data_type, "H.264")) {
		VENC_H264_QBIAS_S qbias;
		qbias.bEnable = rk_param_get_int("video.1:qbias_enable", 0);
		qbias.u32QbiasI = rk_param_get_int("video.1:qbias_i", 0);
		qbias.u32QbiasP = rk_param_get_int("video.1:qbias_p", 0);
		ret = RK_MPI_VENC_SetH264Qbias(VIDEO_PIPE_1, &qbias);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_SetH264Qbias error! ret=%#x\n", ret);
	} else {
		VENC_H265_QBIAS_S qbias;
		qbias.bEnable = rk_param_get_int("video.1:qbias_enable", 0);
		qbias.u32QbiasI = rk_param_get_int("video.1:qbias_i", 0);
		qbias.u32QbiasP = rk_param_get_int("video.1:qbias_p", 0);
		ret = RK_MPI_VENC_SetH265Qbias(VIDEO_PIPE_1, &qbias);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_SetH265Qbias error! ret=%#x\n", ret);
	}

	VENC_FILTER_S pstFilter;
	RK_MPI_VENC_GetFilter(VIDEO_PIPE_1, &pstFilter);
	pstFilter.u32StrengthI = rk_param_get_int("video.1:flt_str_i", 0);
	pstFilter.u32StrengthP = rk_param_get_int("video.1:flt_str_p", 0);
	RK_MPI_VENC_SetFilter(VIDEO_PIPE_1, &pstFilter);

	tmp_rc_quality = rk_param_get_string("video.1:rc_quality", NULL);
	VENC_RC_PARAM_S venc_rc_param;
	RK_MPI_VENC_GetRcParam(VIDEO_PIPE_1, &venc_rc_param);
	if (!strcmp(tmp_output_data_type, "H.264")) {
		if (!strcmp(tmp_rc_quality, "highest")) {
			venc_rc_param.stParamH264.u32MinQp = 10;
		} else if (!strcmp(tmp_rc_quality, "higher")) {
			venc_rc_param.stParamH264.u32MinQp = 15;
		} else if (!strcmp(tmp_rc_quality, "high")) {
			venc_rc_param.stParamH264.u32MinQp = 20;
		} else if (!strcmp(tmp_rc_quality, "medium")) {
			venc_rc_param.stParamH264.u32MinQp = 25;
		} else if (!strcmp(tmp_rc_quality, "low")) {
			venc_rc_param.stParamH264.u32MinQp = 30;
		} else if (!strcmp(tmp_rc_quality, "lower")) {
			venc_rc_param.stParamH264.u32MinQp = 35;
		} else {
			venc_rc_param.stParamH264.u32MinQp = 40;
		}
		venc_rc_param.stParamH264.u32FrmMinIQp = frame_min_i_qp;
		venc_rc_param.stParamH264.u32FrmMinQp = frame_min_qp;
		venc_rc_param.stParamH264.u32FrmMaxIQp = frame_max_i_qp;
		venc_rc_param.stParamH264.u32FrmMaxQp = frame_max_qp;
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		if (!strcmp(tmp_rc_quality, "highest")) {
			venc_rc_param.stParamH265.u32MinQp = 10;
		} else if (!strcmp(tmp_rc_quality, "higher")) {
			venc_rc_param.stParamH265.u32MinQp = 15;
		} else if (!strcmp(tmp_rc_quality, "high")) {
			venc_rc_param.stParamH265.u32MinQp = 20;
		} else if (!strcmp(tmp_rc_quality, "medium")) {
			venc_rc_param.stParamH265.u32MinQp = 25;
		} else if (!strcmp(tmp_rc_quality, "low")) {
			venc_rc_param.stParamH265.u32MinQp = 30;
		} else if (!strcmp(tmp_rc_quality, "lower")) {
			venc_rc_param.stParamH265.u32MinQp = 35;
		} else {
			venc_rc_param.stParamH265.u32MinQp = 40;
		}
		venc_rc_param.stParamH265.u32FrmMinIQp = frame_min_i_qp;
		venc_rc_param.stParamH265.u32FrmMinQp = frame_min_qp;
		venc_rc_param.stParamH265.u32FrmMaxIQp = frame_max_i_qp;
		venc_rc_param.stParamH265.u32FrmMaxQp = frame_max_qp;
	} else {
		LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
		return -1;
	}
	RK_MPI_VENC_SetRcParam(VIDEO_PIPE_1, &venc_rc_param);

	if (!strcmp(tmp_output_data_type, "H.264")) {
		VENC_H264_TRANS_S pstH264Trans;
		RK_MPI_VENC_GetH264Trans(VIDEO_PIPE_1, &pstH264Trans);
		pstH264Trans.bScalingListValid = scalinglist;
		RK_MPI_VENC_SetH264Trans(VIDEO_PIPE_1, &pstH264Trans);
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		VENC_H265_TRANS_S pstH265Trans;
		RK_MPI_VENC_GetH265Trans(VIDEO_PIPE_1, &pstH265Trans);
		pstH265Trans.bScalingListEnabled = scalinglist;
		RK_MPI_VENC_SetH265Trans(VIDEO_PIPE_1, &pstH265Trans);
	}
	VENC_CHN_REF_BUF_SHARE_S stVencChnRefBufShare;
	memset(&stVencChnRefBufShare, 0, sizeof(VENC_CHN_REF_BUF_SHARE_S));
	stVencChnRefBufShare.bEnable = rk_param_get_int("video.1:enable_refer_buffer_share", 0);
	RK_MPI_VENC_SetChnRefBufShareAttr(VIDEO_PIPE_1, &stVencChnRefBufShare);
	if (rotation == 0) {
		RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_1, ROTATION_0);
	} else if (rotation == 90) {
		RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_1, ROTATION_90);
	} else if (rotation == 180) {
		RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_1, ROTATION_180);
	} else if (rotation == 270) {
		RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_1, ROTATION_270);
	}

	VENC_RECV_PIC_PARAM_S stRecvParam;
	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(VIDEO_PIPE_1, &stRecvParam);
	pthread_create(&venc_thread_1, NULL, rkipc_get_venc_lazy, (void *)(intptr_t)1);

	// NOTE: VI->VENC bind moved to rkipc_get_venc_lazy (lazy binding)

	if (!g_enable_vo)
		return 0;
#if HAS_VO
	// VO
	VO_PUB_ATTR_S VoPubAttr;
	VO_VIDEO_LAYER_ATTR_S stLayerAttr;
	VO_CSC_S VideoCSC;
	VO_CHN_ATTR_S VoChnAttr;
	RK_U32 u32DispBufLen;
	memset(&VoPubAttr, 0, sizeof(VO_PUB_ATTR_S));
	memset(&stLayerAttr, 0, sizeof(VO_VIDEO_LAYER_ATTR_S));
	memset(&VideoCSC, 0, sizeof(VO_CSC_S));
	memset(&VoChnAttr, 0, sizeof(VoChnAttr));

	if (g_vo_dev_id == 0) {
		VoPubAttr.enIntfType = VO_INTF_HDMI;
		VoPubAttr.enIntfSync = VO_OUTPUT_1080P60;
	} else {
		VoPubAttr.enIntfType = VO_INTF_MIPI;
		VoPubAttr.enIntfSync = VO_OUTPUT_DEFAULT;
	}
	ret = RK_MPI_VO_SetPubAttr(g_vo_dev_id, &VoPubAttr);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("RK_MPI_VO_SetPubAttr %x\n", ret);
		return ret;
	}
	LOG_DEBUG("RK_MPI_VO_SetPubAttr success\n");

	ret = RK_MPI_VO_Enable(g_vo_dev_id);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("RK_MPI_VO_Enable err is %x\n", ret);
		return ret;
	}
	LOG_DEBUG("RK_MPI_VO_Enable success\n");

	ret = RK_MPI_VO_GetLayerDispBufLen(VoLayer, &u32DispBufLen);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("Get display buf len failed with error code %d!\n", ret);
		return ret;
	}
	LOG_DEBUG("Get VoLayer %d disp buf len is %d.\n", VoLayer, u32DispBufLen);
	u32DispBufLen = 3;
	ret = RK_MPI_VO_SetLayerDispBufLen(VoLayer, u32DispBufLen);
	if (ret != RK_SUCCESS) {
		return ret;
	}
	LOG_DEBUG("Agin Get VoLayer %d disp buf len is %d.\n", VoLayer, u32DispBufLen);

	/* get vo attribute*/
	ret = RK_MPI_VO_GetPubAttr(g_vo_dev_id, &VoPubAttr);
	if (ret) {
		LOG_ERROR("RK_MPI_VO_GetPubAttr fail!\n");
		return ret;
	}
	LOG_DEBUG("RK_MPI_VO_GetPubAttr success\n");
	if ((VoPubAttr.stSyncInfo.u16Hact == 0) || (VoPubAttr.stSyncInfo.u16Vact == 0)) {
		if (g_vo_dev_id == RK3588_VO_DEV_HDMI) {
			VoPubAttr.stSyncInfo.u16Hact = 1920;
			VoPubAttr.stSyncInfo.u16Vact = 1080;
		} else {
			VoPubAttr.stSyncInfo.u16Hact = 1080;
			VoPubAttr.stSyncInfo.u16Vact = 1920;
		}
	}

	stLayerAttr.stDispRect.s32X = 0;
	stLayerAttr.stDispRect.s32Y = 0;
	stLayerAttr.stDispRect.u32Width = VoPubAttr.stSyncInfo.u16Hact;
	stLayerAttr.stDispRect.u32Height = VoPubAttr.stSyncInfo.u16Vact;
	stLayerAttr.stImageSize.u32Width = VoPubAttr.stSyncInfo.u16Hact;
	stLayerAttr.stImageSize.u32Height = VoPubAttr.stSyncInfo.u16Vact;
	LOG_DEBUG("stLayerAttr W=%d, H=%d\n", stLayerAttr.stDispRect.u32Width,
	          stLayerAttr.stDispRect.u32Height);

	stLayerAttr.u32DispFrmRt = 25;
	stLayerAttr.enPixFormat = RK_FMT_RGB888;
	VideoCSC.enCscMatrix = VO_CSC_MATRIX_IDENTITY;
	VideoCSC.u32Contrast = 50;
	VideoCSC.u32Hue = 50;
	VideoCSC.u32Luma = 50;
	VideoCSC.u32Satuature = 50;
	RK_S32 u32VoChn = 0;

	/*bind layer0 to device hd0*/
	ret = RK_MPI_VO_BindLayer(VoLayer, g_vo_dev_id, VO_LAYER_MODE_GRAPHIC);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("RK_MPI_VO_BindLayer VoLayer = %d error\n", VoLayer);
		return ret;
	}
	LOG_DEBUG("RK_MPI_VO_BindLayer success\n");

	ret = RK_MPI_VO_SetLayerAttr(VoLayer, &stLayerAttr);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("RK_MPI_VO_SetLayerAttr VoLayer = %d error\n", VoLayer);
		return ret;
	}
	LOG_DEBUG("RK_MPI_VO_SetLayerAttr success\n");

	ret = RK_MPI_VO_EnableLayer(VoLayer);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("RK_MPI_VO_EnableLayer VoLayer = %d error\n", VoLayer);
		return ret;
	}
	LOG_DEBUG("RK_MPI_VO_EnableLayer success\n");

	ret = RK_MPI_VO_SetLayerCSC(VoLayer, &VideoCSC);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("RK_MPI_VO_SetLayerCSC error\n");
		return ret;
	}
	LOG_DEBUG("RK_MPI_VO_SetLayerCSC success\n");

	ret = RK_MPI_VO_EnableChn(RK3588_VOP_LAYER_CLUSTER0, u32VoChn);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("create RK3588_VOP_LAYER_CLUSTER0 layer %d ch vo failed!\n", u32VoChn);
		return ret;
	}
	LOG_DEBUG("RK_MPI_VO_EnableChn success\n");

	VoChnAttr.bDeflicker = RK_FALSE;
	VoChnAttr.u32Priority = 1;
	VoChnAttr.stRect.s32X = 0;
	VoChnAttr.stRect.s32Y = 0;
	VoChnAttr.stRect.u32Width = stLayerAttr.stDispRect.u32Width;
	VoChnAttr.stRect.u32Height = stLayerAttr.stDispRect.u32Height;
	ret = RK_MPI_VO_SetChnAttr(VoLayer, 0, &VoChnAttr);

	pthread_t thread_id;
	pthread_create(&thread_id, NULL, get_vi_send_vo, NULL);
#endif

	return 0;
}

int rkipc_pipe_1_deinit() {
	int ret;
	// unbind only if still bound (lazy binding)
	if (pipe_1_bound) {
		MPP_CHN_S lvi, lvenc;
		lvi.enModId = RK_ID_VI;
		lvi.s32DevId = 0;
		lvi.s32ChnId = VIDEO_PIPE_1;
		lvenc.enModId = RK_ID_VENC;
		lvenc.s32DevId = 0;
		lvenc.s32ChnId = VIDEO_PIPE_1;
		ret = RK_MPI_SYS_UnBind(&lvi, &lvenc);
		if (ret)
			LOG_ERROR("Unbind VI and VENC error! ret=%#x\n", ret);
		pipe_1_bound = 0;
	}
	// VENC
	ret = RK_MPI_VENC_StopRecvFrame(VIDEO_PIPE_1);
	ret |= RK_MPI_VENC_DestroyChn(VIDEO_PIPE_1);
	if (ret)
		LOG_ERROR("ERROR: Destroy VENC error! ret=%#x\n", ret);
	else
		LOG_DEBUG("RK_MPI_VENC_DestroyChn success\n");
	// VI
	ret = RK_MPI_VI_DisableChn(pipe_id_, VIDEO_PIPE_1);
	if (ret)
		LOG_ERROR("ERROR: Destroy VI error! ret=%#x\n", ret);

	return 0;
}

int rkipc_pipe_2_init() {
	int ret;
	int video_width = rk_param_get_int("video.2:width", -1);
	int video_height = rk_param_get_int("video.2:height", -1);
	int buf_cnt = 2;

	// VI
	VI_CHN_ATTR_S vi_chn_attr;
	memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
	if (enable_npu) // ensure vi and ivs have two buffer ping-pong
		buf_cnt += 1;
	vi_chn_attr.stIspOpt.u32BufCount = buf_cnt;
	vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	vi_chn_attr.stIspOpt.stMaxSize.u32Width = rk_param_get_int("video.2:max_width", 960);
	vi_chn_attr.stIspOpt.stMaxSize.u32Height = rk_param_get_int("video.2:max_height", 540);
	vi_chn_attr.stSize.u32Width = video_width;
	vi_chn_attr.stSize.u32Height = video_height;
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	vi_chn_attr.u32Depth = 0;
	if (enable_npu)
		vi_chn_attr.u32Depth += 1;
	ret = RK_MPI_VI_SetChnAttr(pipe_id_, VIDEO_PIPE_2, &vi_chn_attr);
	if (ret) {
		LOG_ERROR("ERROR: create VI error! ret=%d\n", ret);
		return ret;
	}

	ret = RK_MPI_VI_EnableChn(pipe_id_, VIDEO_PIPE_2);
	if (ret) {
		LOG_ERROR("ERROR: create VI error! ret=%d\n", ret);
		return ret;
	}

	if (enable_ivs) {
		rkipc_ivs_init();
		// bind
		vi_chn.enModId = RK_ID_VI;
		vi_chn.s32DevId = 0;
		vi_chn.s32ChnId = VIDEO_PIPE_2;
		ivs_chn.enModId = RK_ID_IVS;
		ivs_chn.s32DevId = 0;
		ivs_chn.s32ChnId = 0;
		ret = RK_MPI_SYS_Bind(&vi_chn, &ivs_chn);
		if (ret)
			LOG_ERROR("Bind VI and IVS error! ret=%#x\n", ret);
		else
			LOG_DEBUG("Bind VI and IVS success\n");
	}
	if (enable_npu) {
		pthread_create(&get_vi_2_send_thread, NULL, rkipc_get_vi_2_send, NULL);
		rkipc_osd_draw_nn_init();
	}
}

int rkipc_pipe_2_deinit() {
	int ret;
	if (enable_npu) {
		rkipc_osd_draw_nn_deinit();
		pthread_join(get_vi_2_send_thread, NULL);
	}
	if (enable_ivs) {
		// unbind
		vi_chn.enModId = RK_ID_VI;
		vi_chn.s32DevId = 0;
		vi_chn.s32ChnId = VIDEO_PIPE_2;
		ivs_chn.enModId = RK_ID_IVS;
		ivs_chn.s32DevId = 0;
		ivs_chn.s32ChnId = 0;
		ret = RK_MPI_SYS_UnBind(&vi_chn, &ivs_chn);
		if (ret)
			LOG_ERROR("Unbind VI and IVS error! ret=%#x\n", ret);
		else
			LOG_DEBUG("Unbind VI and IVS success\n");
		rkipc_ivs_deinit();
	}
	ret = RK_MPI_VI_DisableChn(pipe_id_, VIDEO_PIPE_2);
	if (ret)
		LOG_ERROR("ERROR: Destroy VI error! ret=%#x\n", ret);

	return 0;
}

int rkipc_pipe_jpeg_init() {
	// jpeg resolution same to video.0
	int ret;
	int video_width = rk_param_get_int("video.jpeg:width", 1920);
	int video_height = rk_param_get_int("video.jpeg:height", 1080);
	int video_max_height = rk_param_get_int("video.0:max_height", -1);
	int rotation = rk_param_get_int("video.source:rotation", 0);
	// VENC[3] init
	VENC_CHN_ATTR_S jpeg_chn_attr;
	memset(&jpeg_chn_attr, 0, sizeof(jpeg_chn_attr));
	jpeg_chn_attr.stVencAttr.enType = RK_VIDEO_ID_JPEG;
	jpeg_chn_attr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	jpeg_chn_attr.stVencAttr.u32MaxPicWidth = rk_param_get_int("video.0:max_width", 2560);
	jpeg_chn_attr.stVencAttr.u32MaxPicHeight = rk_param_get_int("video.0:max_height", 1440);
	jpeg_chn_attr.stVencAttr.u32PicWidth = video_width;
	jpeg_chn_attr.stVencAttr.u32PicHeight = video_height;
	jpeg_chn_attr.stVencAttr.u32VirWidth = video_width;
	jpeg_chn_attr.stVencAttr.u32VirHeight = video_height;
	jpeg_chn_attr.stVencAttr.u32StreamBufCnt = 2;
	jpeg_chn_attr.stVencAttr.u32BufSize = rk_param_get_int("video.jpeg:jpeg_buffer_size", 204800);
	// jpeg_chn_attr.stVencAttr.u32Depth = 1;
	ret = RK_MPI_VENC_CreateChn(JPEG_VENC_CHN, &jpeg_chn_attr);
	if (ret) {
		LOG_ERROR("ERROR: create VENC error! ret=%d\n", ret);
		return -1;
	}
	VENC_JPEG_PARAM_S stJpegParam;
	memset(&stJpegParam, 0, sizeof(stJpegParam));
	stJpegParam.u32Qfactor = rk_param_get_int("video.jpeg:jpeg_qfactor", 70);
	RK_MPI_VENC_SetJpegParam(JPEG_VENC_CHN, &stJpegParam);
	if (rotation == 0) {
		RK_MPI_VENC_SetChnRotation(JPEG_VENC_CHN, ROTATION_0);
	} else if (rotation == 90) {
		RK_MPI_VENC_SetChnRotation(JPEG_VENC_CHN, ROTATION_90);
	} else if (rotation == 180) {
		RK_MPI_VENC_SetChnRotation(JPEG_VENC_CHN, ROTATION_180);
	} else if (rotation == 270) {
		RK_MPI_VENC_SetChnRotation(JPEG_VENC_CHN, ROTATION_270);
	}

	VENC_CHN_BUF_WRAP_S stVencChnBufWrap;
	memset(&stVencChnBufWrap, 0, sizeof(stVencChnBufWrap));
	if (enable_wrap) {
		stVencChnBufWrap.bEnable = enable_wrap;
		stVencChnBufWrap.u32BufLine =
		    rk_param_get_int("video.source:buffer_line", video_max_height / 4);
		if (stVencChnBufWrap.u32BufLine < 128)
			stVencChnBufWrap.u32BufLine = video_max_height;
		RK_MPI_VENC_SetChnBufWrapAttr(JPEG_VENC_CHN, &stVencChnBufWrap);
	}

	VENC_COMBO_ATTR_S stComboAttr;
	memset(&stComboAttr, 0, sizeof(VENC_COMBO_ATTR_S));
	stComboAttr.bEnable = RK_TRUE;
	stComboAttr.s32ChnId = VIDEO_PIPE_0;
	RK_MPI_VENC_SetComboAttr(JPEG_VENC_CHN, &stComboAttr);

	VENC_RECV_PIC_PARAM_S stRecvParam;
	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = 1;
	RK_MPI_VENC_StartRecvFrame(JPEG_VENC_CHN,
	                           &stRecvParam); // must, for no streams callback running failed

	pthread_create(&jpeg_venc_thread_id, NULL, rkipc_get_jpeg, NULL);
	if (rk_param_get_int("video.jpeg:enable_cycle_snapshot", 0)) {
		cycle_snapshot_flag = 1;
		pthread_create(&cycle_snapshot_thread_id, NULL, rkipc_cycle_snapshot, NULL);
	}

	return ret;
}

int rkipc_pipe_jpeg_deinit() {
	int ret = 0;
	ret = RK_MPI_VENC_StopRecvFrame(JPEG_VENC_CHN);
	ret |= RK_MPI_VENC_DestroyChn(JPEG_VENC_CHN);
	if (ret)
		LOG_ERROR("ERROR: Destroy VENC error! ret=%#x\n", ret);
	else
		LOG_INFO("RK_MPI_VENC_DestroyChn success\n");

	return ret;
}

int rkipc_ivs_init() {
	int ret;
	int video_width = rk_param_get_int("video.2:width", -1);
	int video_height = rk_param_get_int("video.2:height", -1);
	int buf_cnt = 2;
	int smear = rk_param_get_int("ivs:smear", 1);
	int weightp = rk_param_get_int("ivs:weightp", 1);
	int md = rk_param_get_int("ivs:md", 0);
	int od = rk_param_get_int("ivs:od", 0);
	if (!smear && !weightp && !md && !od) {
		LOG_INFO("no pp function enabled! end\n");
		return -1;
	}

	// IVS
	IVS_CHN_ATTR_S attr;
	memset(&attr, 0, sizeof(attr));
	attr.enMode = IVS_MODE_MD_OD;
	attr.u32PicWidth = video_width;
	attr.u32PicHeight = video_height;
	attr.enPixelFormat = RK_FMT_YUV420SP;
	attr.s32Gop = rk_param_get_int("video.0:gop", 30);
	attr.bSmearEnable = smear;
	attr.bWeightpEnable = weightp;
	attr.bMDEnable = md;
	attr.s32MDInterval = 5;
	attr.bMDNightMode = RK_TRUE;
	attr.u32MDSensibility = rk_param_get_int("ivs:md_sensibility", 3);
	attr.bODEnable = od;
	attr.s32ODInterval = 1;
	attr.s32ODPercent = 6;
	ret = RK_MPI_IVS_CreateChn(0, &attr);
	if (ret) {
		LOG_ERROR("ERROR: RK_MPI_IVS_CreateChn error! ret=%#x\n", ret);
		return -1;
	}

	IVS_MD_ATTR_S stMdAttr;
	memset(&stMdAttr, 0, sizeof(stMdAttr));
	ret = RK_MPI_IVS_GetMdAttr(0, &stMdAttr);
	if (ret) {
		LOG_ERROR("ERROR: RK_MPI_IVS_GetMdAttr error! ret=%#x\n", ret);
		return -1;
	}
	stMdAttr.s32ThreshSad = 40;
	stMdAttr.s32ThreshMove = 2;
	stMdAttr.s32SwitchSad = 0;
	ret = RK_MPI_IVS_SetMdAttr(0, &stMdAttr);
	if (ret) {
		LOG_ERROR("ERROR: RK_MPI_IVS_SetMdAttr error! ret=%#x\n", ret);
		return -1;
	}

	if (md == 1 || od == 1)
		pthread_create(&get_ivs_result_thread, NULL, rkipc_ivs_get_results, NULL);

	return 0;
}

int rkipc_ivs_deinit() {
	int ret;
	pthread_join(get_ivs_result_thread, NULL);
	// IVS
	ret = RK_MPI_IVS_DestroyChn(0);
	if (ret)
		LOG_ERROR("ERROR: Destroy IVS error! ret=%#x\n", ret);
	else
		LOG_DEBUG("RK_MPI_IVS_DestroyChn success\n");

	return 0;
}

static RK_U8 rgn_color_lut_0_left_value[4] = {0x03, 0xf, 0x3f, 0xff};
static RK_U8 rgn_color_lut_0_right_value[4] = {0xc0, 0xf0, 0xfc, 0xff};
static RK_U8 rgn_color_lut_1_left_value[4] = {0x02, 0xa, 0x2a, 0xaa};
static RK_U8 rgn_color_lut_1_right_value[4] = {0x80, 0xa0, 0xa8, 0xaa};
RK_S32 draw_rect_2bpp(RK_U8 *buffer, RK_U32 width, RK_U32 height, int rgn_x, int rgn_y, int rgn_w,
                      int rgn_h, int line_pixel, COLOR_INDEX_E color_index) {
	int i;
	RK_U8 *ptr = buffer;
	RK_U8 value = 0;
	if (color_index == RGN_COLOR_LUT_INDEX_0)
		value = 0xff;
	if (color_index == RGN_COLOR_LUT_INDEX_1)
		value = 0xaa;

	if (line_pixel > 4) {
		printf("line_pixel > 4, not support\n", line_pixel);
		return -1;
	}

	// printf("YUV %dx%d, rgn (%d,%d,%d,%d), line pixel %d\n", width, height, rgn_x, rgn_y, rgn_w,
	// rgn_h, line_pixel); draw top line
	ptr += (width * rgn_y + rgn_x) >> 2;
	for (i = 0; i < line_pixel; i++) {
		memset(ptr, value, (rgn_w + 3) >> 2);
		ptr += width >> 2;
	}
	// draw letft/right line
	for (i = 0; i < (rgn_h - line_pixel * 2); i++) {
		if (color_index == RGN_COLOR_LUT_INDEX_1) {
			*ptr = rgn_color_lut_1_left_value[line_pixel - 1];
			*(ptr + ((rgn_w + 3) >> 2)) = rgn_color_lut_1_right_value[line_pixel - 1];
		} else {
			*ptr = rgn_color_lut_0_left_value[line_pixel - 1];
			*(ptr + ((rgn_w + 3) >> 2)) = rgn_color_lut_0_right_value[line_pixel - 1];
		}
		ptr += width >> 2;
	}
	// draw bottom line
	for (i = 0; i < line_pixel; i++) {
		memset(ptr, value, (rgn_w + 3) >> 2);
		ptr += width >> 2;
	}
	return 0;
}

static void *rkipc_get_nn_update_osd(void *arg) {
	g_nn_osd_run_ = 1;
	LOG_DEBUG("#Start %s thread, arg:%p\n", __func__, arg);
	prctl(PR_SET_NAME, "RkipcNpuOsd", 0, 0, 0);

	int ret = 0;
	int line_pixel = 2;
	int change_to_nothing_flag = 0;
	int video_width = 0;
	int video_height = 0;
	int rotation = 0;
	long long last_ba_result_time;
	RockIvaBaResult ba_result;
	RockIvaBaObjectInfo *object;
	RGN_HANDLE RgnHandle = DRAW_NN_OSD_ID;
	RGN_CANVAS_INFO_S stCanvasInfo;

	memset(&stCanvasInfo, 0, sizeof(RGN_CANVAS_INFO_S));
	memset(&ba_result, 0, sizeof(ba_result));
	while (g_nn_osd_run_) {
		usleep(40 * 1000);
		rotation = rk_param_get_int("video.source:rotation", 0);
		if (rotation == 90 || rotation == 270) {
			video_width = rk_param_get_int("video.0:height", -1);
			video_height = rk_param_get_int("video.0:width", -1);
		} else {
			video_width = rk_param_get_int("video.0:width", -1);
			video_height = rk_param_get_int("video.0:height", -1);
		}
		ret = rkipc_rknn_object_get(&ba_result);
		// LOG_DEBUG("ret is %d, ba_result.objNum is %d\n", ret, ba_result.objNum);

		if ((ret == -1) && (rkipc_get_curren_time_ms() - last_ba_result_time > 300))
			ba_result.objNum = 0;
		if (ret == 0)
			last_ba_result_time = rkipc_get_curren_time_ms();

		ret = RK_MPI_RGN_GetCanvasInfo(RgnHandle, &stCanvasInfo);
		if (ret != RK_SUCCESS) {
			RK_LOGE("RK_MPI_RGN_GetCanvasInfo failed with %#x!", ret);
			continue;
		}
		if ((stCanvasInfo.stSize.u32Width != UPALIGNTO16(video_width)) ||
		    (stCanvasInfo.stSize.u32Height != UPALIGNTO16(video_height))) {
			LOG_WARN("canvas is %d*%d, not equal %d*%d, maybe in the process of switching,"
			         "skip this time\n",
			         stCanvasInfo.stSize.u32Width, stCanvasInfo.stSize.u32Height,
			         UPALIGNTO16(video_width), UPALIGNTO16(video_height));
			continue;
		}
		memset((void *)stCanvasInfo.u64VirAddr, 0,
		       stCanvasInfo.u32VirWidth * stCanvasInfo.u32VirHeight >> 2);
		// draw
		for (int i = 0; i < ba_result.objNum; i++) {
			int x, y, w, h;
			object = &ba_result.triggerObjects[i];
			// LOG_INFO("topLeft:[%d,%d], bottomRight:[%d,%d],"
			// 			"objId is %d, frameId is %d, score is %d, type is %d\n",
			// 			object->objInfo.rect.topLeft.x, object->objInfo.rect.topLeft.y,
			// 			object->objInfo.rect.bottomRight.x,
			// 			object->objInfo.rect.bottomRight.y, object->objInfo.objId,
			// 			object->objInfo.frameId, object->objInfo.score, object->objInfo.type);
			x = video_width * object->objInfo.rect.topLeft.x / 10000;
			y = video_height * object->objInfo.rect.topLeft.y / 10000;
			w = video_width *
			    (object->objInfo.rect.bottomRight.x - object->objInfo.rect.topLeft.x) / 10000;
			h = video_height *
			    (object->objInfo.rect.bottomRight.y - object->objInfo.rect.topLeft.y) / 10000;
			x = x / 16 * 16;
			y = y / 16 * 16;
			w = (w + 3) / 16 * 16;
			h = (h + 3) / 16 * 16;

			while (x + w + line_pixel >= video_width) {
				w -= 8;
			}
			while (y + h + line_pixel >= video_height) {
				h -= 8;
			}
			if (x < 0 || y < 0 || w <= 0 || h <= 0) {
				continue;
			}
			// LOG_DEBUG("i is %d, x,y,w,h is %d,%d,%d,%d\n", i, x, y, w, h);
			if (object->objInfo.type == ROCKIVA_OBJECT_TYPE_PERSON) {
				draw_rect_2bpp((RK_U8 *)stCanvasInfo.u64VirAddr, stCanvasInfo.u32VirWidth,
				               stCanvasInfo.u32VirHeight, x, y, w, h, line_pixel,
				               RGN_COLOR_LUT_INDEX_0);
			} else if (object->objInfo.type == ROCKIVA_OBJECT_TYPE_FACE) {
				draw_rect_2bpp((RK_U8 *)stCanvasInfo.u64VirAddr, stCanvasInfo.u32VirWidth,
				               stCanvasInfo.u32VirHeight, x, y, w, h, line_pixel,
				               RGN_COLOR_LUT_INDEX_0);
			}
			// LOG_INFO("draw rect time-consuming is %lld\n",(rkipc_get_curren_time_ms() -
			// 	last_ba_result_time));
			// LOG_INFO("triggerRules is %d, ruleID is %d, triggerType is %d\n",
			// 			object->triggerRules,
			// 			object->firstTrigger.ruleID,
			// 			object->firstTrigger.triggerType);
		}
		ret = RK_MPI_RGN_UpdateCanvas(RgnHandle);
		if (ret != RK_SUCCESS) {
			RK_LOGE("RK_MPI_RGN_UpdateCanvas failed with %#x!", ret);
			continue;
		}
	}

	return 0;
}

int rkipc_osd_draw_nn_init() {
	LOG_DEBUG("start\n");
	int ret = 0;
	RGN_HANDLE RgnHandle = DRAW_NN_OSD_ID;
	RGN_ATTR_S stRgnAttr;
	MPP_CHN_S stMppChn;
	RGN_CHN_ATTR_S stRgnChnAttr;
	BITMAP_S stBitmap;
	int rotation = rk_param_get_int("video.source:rotation", 0);

	// create overlay regions
	memset(&stRgnAttr, 0, sizeof(stRgnAttr));
	stRgnAttr.enType = OVERLAY_RGN;
	stRgnAttr.unAttr.stOverlay.enPixelFmt = RK_FMT_2BPP;
	stRgnAttr.unAttr.stOverlay.u32CanvasNum = 1;
	if (rotation == 90 || rotation == 270) {
		stRgnAttr.unAttr.stOverlay.stSize.u32Width = rk_param_get_int("video.0:max_height", -1);
		stRgnAttr.unAttr.stOverlay.stSize.u32Height = rk_param_get_int("video.0:max_width", -1);
	} else {
		stRgnAttr.unAttr.stOverlay.stSize.u32Width = rk_param_get_int("video.0:max_width", -1);
		stRgnAttr.unAttr.stOverlay.stSize.u32Height = rk_param_get_int("video.0:max_height", -1);
	}
	ret = RK_MPI_RGN_Create(RgnHandle, &stRgnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_Create (%d) failed with %#x\n", RgnHandle, ret);
		RK_MPI_RGN_Destroy(RgnHandle);
		return RK_FAILURE;
	}
	LOG_DEBUG("The handle: %d, create success\n", RgnHandle);
	// after malloc max size, it needs to be set to the actual size
	if (rotation == 90 || rotation == 270) {
		stRgnAttr.unAttr.stOverlay.stSize.u32Width = rk_param_get_int("video.0:height", -1);
		stRgnAttr.unAttr.stOverlay.stSize.u32Height = rk_param_get_int("video.0:width", -1);
	} else {
		stRgnAttr.unAttr.stOverlay.stSize.u32Width = rk_param_get_int("video.0:width", -1);
		stRgnAttr.unAttr.stOverlay.stSize.u32Height = rk_param_get_int("video.0:height", -1);
	}
	ret = RK_MPI_RGN_SetAttr(RgnHandle, &stRgnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_SetAttr (%d) failed with %#x!", RgnHandle, ret);
		return RK_FAILURE;
	}

	// display overlay regions to venc groups
	memset(&stRgnChnAttr, 0, sizeof(stRgnChnAttr));
	stRgnChnAttr.bShow = RK_TRUE;
	stRgnChnAttr.enType = OVERLAY_RGN;
	stRgnChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = 0;
	stRgnChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = 0;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32BgAlpha = 0;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32FgAlpha = 255;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32Layer = DRAW_NN_OSD_ID;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32ColorLUT[RGN_COLOR_LUT_INDEX_0] = BLUE_COLOR;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32ColorLUT[RGN_COLOR_LUT_INDEX_1] = RED_COLOR;
	stMppChn.enModId = RK_ID_VENC;
	stMppChn.s32DevId = 0;
	stMppChn.s32ChnId = DRAW_NN_VENC_CHN_ID;
	ret = RK_MPI_RGN_AttachToChn(RgnHandle, &stMppChn, &stRgnChnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_AttachToChn (%d) to venc0 failed with %#x\n", RgnHandle, ret);
		return RK_FAILURE;
	}
	LOG_DEBUG("RK_MPI_RGN_AttachToChn to venc0 success\n");
	pthread_create(&get_nn_update_osd_thread_id, NULL, rkipc_get_nn_update_osd, NULL);
	LOG_DEBUG("end\n");

	return ret;
}

int rkipc_osd_draw_nn_deinit() {
	LOG_DEBUG("%s\n", __func__);
	int ret = 0;
	if (g_nn_osd_run_) {
		g_nn_osd_run_ = 0;
		pthread_join(get_nn_update_osd_thread_id, NULL);
	}
	// Detach osd from chn
	MPP_CHN_S stMppChn;
	RGN_HANDLE RgnHandle = DRAW_NN_OSD_ID;
	stMppChn.enModId = RK_ID_VENC;
	stMppChn.s32DevId = 0;
	stMppChn.s32ChnId = DRAW_NN_VENC_CHN_ID;
	ret = RK_MPI_RGN_DetachFromChn(RgnHandle, &stMppChn);
	if (RK_SUCCESS != ret)
		LOG_ERROR("RK_MPI_RGN_DetachFrmChn (%d) to venc0 failed with %#x\n", RgnHandle, ret);

	// destory region
	ret = RK_MPI_RGN_Destroy(RgnHandle);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_Destroy [%d] failed with %#x\n", RgnHandle, ret);
	}
	LOG_DEBUG("Destory handle:%d success\n", RgnHandle);

	return ret;
}

int rkipc_osd_draw_nn_change() {
	LOG_INFO("%s\n", __func__);
	int ret = 0;
	int rotation = rk_param_get_int("video.source:rotation", 0);
	MPP_CHN_S stMppChn;
	RGN_ATTR_S stRgnAttr;
	RGN_CHN_ATTR_S stRgnChnAttr;
	RGN_HANDLE RgnHandle = DRAW_NN_OSD_ID;
	stMppChn.enModId = RK_ID_VENC;
	stMppChn.s32DevId = 0;
	stMppChn.s32ChnId = DRAW_NN_VENC_CHN_ID;
	ret = RK_MPI_RGN_DetachFromChn(RgnHandle, &stMppChn);
	if (RK_SUCCESS != ret)
		LOG_ERROR("RK_MPI_RGN_DetachFrmChn (%d) to venc0 failed with %#x\n", RgnHandle, ret);
	ret = RK_MPI_RGN_GetAttr(RgnHandle, &stRgnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_GetAttr (%d) failed with %#x!", RgnHandle, ret);
		return RK_FAILURE;
	}
	if (rotation == 90 || rotation == 270) {
		stRgnAttr.unAttr.stOverlay.stSize.u32Width = rk_param_get_int("video.0:height", -1);
		stRgnAttr.unAttr.stOverlay.stSize.u32Height = rk_param_get_int("video.0:width", -1);
	} else {
		stRgnAttr.unAttr.stOverlay.stSize.u32Width = rk_param_get_int("video.0:width", -1);
		stRgnAttr.unAttr.stOverlay.stSize.u32Height = rk_param_get_int("video.0:height", -1);
	}
	ret = RK_MPI_RGN_SetAttr(RgnHandle, &stRgnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_SetAttr (%d) failed with %#x!", RgnHandle, ret);
		return RK_FAILURE;
	}

	memset(&stRgnChnAttr, 0, sizeof(stRgnChnAttr));
	stRgnChnAttr.bShow = RK_TRUE;
	stRgnChnAttr.enType = OVERLAY_RGN;
	stRgnChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = 0;
	stRgnChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = 0;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32BgAlpha = 0;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32FgAlpha = 255;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32Layer = DRAW_NN_OSD_ID;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32ColorLUT[RGN_COLOR_LUT_INDEX_0] = BLUE_COLOR;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32ColorLUT[RGN_COLOR_LUT_INDEX_1] = RED_COLOR;
	ret = RK_MPI_RGN_AttachToChn(RgnHandle, &stMppChn, &stRgnChnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_AttachToChn (%d) failed with %#x!", RgnHandle, ret);
		return RK_FAILURE;
	}

	return ret;
}

// export API
int rk_video_get_gop(int stream_id, int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:gop", stream_id);
	*value = rk_param_get_int(entry, -1);

	return 0;
}

int rk_video_set_gop(int stream_id, int value) {
	char entry[128] = {'\0'};
	VENC_CHN_ATTR_S venc_chn_attr;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	RK_MPI_VENC_GetChnAttr(stream_id, &venc_chn_attr);
	snprintf(entry, 127, "video.%d:output_data_type", stream_id);
	tmp_output_data_type = rk_param_get_string(entry, "H.264");
	snprintf(entry, 127, "video.%d:rc_mode", stream_id);
	tmp_rc_mode = rk_param_get_string(entry, "CBR");
	if (!strcmp(tmp_output_data_type, "H.264")) {
		if (!strcmp(tmp_rc_mode, "CBR"))
			venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = value;
		else
			venc_chn_attr.stRcAttr.stH264Vbr.u32Gop = value;
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		if (!strcmp(tmp_rc_mode, "CBR"))
			venc_chn_attr.stRcAttr.stH265Cbr.u32Gop = value;
		else
			venc_chn_attr.stRcAttr.stH265Vbr.u32Gop = value;
	} else {
		LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
		return -1;
	}
	RK_MPI_VENC_SetChnAttr(stream_id, &venc_chn_attr);
	snprintf(entry, 127, "video.%d:gop", stream_id);
	rk_param_set_int(entry, value);

	return 0;
}

int rk_video_get_max_rate(int stream_id, int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:max_rate", stream_id);
	*value = rk_param_get_int(entry, -1);

	return 0;
}

int rk_video_set_max_rate(int stream_id, int value) {
	VENC_CHN_ATTR_S venc_chn_attr;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	RK_MPI_VENC_GetChnAttr(stream_id, &venc_chn_attr);
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:output_data_type", stream_id);
	tmp_output_data_type = rk_param_get_string(entry, "H.264");
	snprintf(entry, 127, "video.%d:rc_mode", stream_id);
	tmp_rc_mode = rk_param_get_string(entry, "CBR");
	if (!strcmp(tmp_output_data_type, "H.264")) {
		if (!strcmp(tmp_rc_mode, "CBR")) {
			venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = value;
		} else {
			venc_chn_attr.stRcAttr.stH264Vbr.u32MinBitRate = value / 3;
			venc_chn_attr.stRcAttr.stH264Vbr.u32BitRate = value / 3 * 2;
			venc_chn_attr.stRcAttr.stH264Vbr.u32MaxBitRate = value;
		}
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		if (!strcmp(tmp_rc_mode, "CBR")) {
			venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = value;
		} else {
			venc_chn_attr.stRcAttr.stH265Vbr.u32MinBitRate = value / 3;
			venc_chn_attr.stRcAttr.stH265Vbr.u32BitRate = value / 3 * 2;
			venc_chn_attr.stRcAttr.stH265Vbr.u32MaxBitRate = value;
		}
	} else {
		LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
		return -1;
	}
	RK_MPI_VENC_SetChnAttr(stream_id, &venc_chn_attr);
	snprintf(entry, 127, "video.%d:max_rate", stream_id);
	rk_param_set_int(entry, value);
	snprintf(entry, 127, "video.%d:mid_rate", stream_id);
	rk_param_set_int(entry, value / 3 * 2);
	snprintf(entry, 127, "video.%d:min_rate", stream_id);
	rk_param_set_int(entry, value / 3);

	return 0;
}

int rk_video_get_RC_mode(int stream_id, const char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:rc_mode", stream_id);
	*value = rk_param_get_string(entry, "CBR");

	return 0;
}

int rk_video_set_RC_mode(int stream_id, const char *value) {
	char entry_output_data_type[128] = {'\0'};
	char entry_gop[128] = {'\0'};
	char entry_mid_rate[128] = {'\0'};
	char entry_max_rate[128] = {'\0'};
	char entry_min_rate[128] = {'\0'};
	char entry_rc_mode[128] = {'\0'};
	snprintf(entry_output_data_type, 127, "video.%d:output_data_type", stream_id);
	snprintf(entry_gop, 127, "video.%d:gop", stream_id);
	snprintf(entry_mid_rate, 127, "video.%d:mid_rate", stream_id);
	snprintf(entry_max_rate, 127, "video.%d:max_rate", stream_id);
	snprintf(entry_min_rate, 127, "video.%d:min_rate", stream_id);
	snprintf(entry_rc_mode, 127, "video.%d:rc_mode", stream_id);

	VENC_CHN_ATTR_S venc_chn_attr;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	RK_MPI_VENC_GetChnAttr(stream_id, &venc_chn_attr);
	tmp_output_data_type = rk_param_get_string(entry_output_data_type, "H.264");
	if (!strcmp(tmp_output_data_type, "H.264")) {
		if (!strcmp(value, "CBR")) {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
			venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = rk_param_get_int(entry_gop, -1);
			venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = rk_param_get_int(entry_max_rate, -1);
		} else {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
			venc_chn_attr.stRcAttr.stH264Vbr.u32Gop = rk_param_get_int(entry_gop, -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32BitRate = rk_param_get_int(entry_mid_rate, -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32MaxBitRate = rk_param_get_int(entry_max_rate, -1);
			venc_chn_attr.stRcAttr.stH264Vbr.u32MinBitRate = rk_param_get_int(entry_min_rate, -1);
		}
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		if (!strcmp(value, "CBR")) {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
			venc_chn_attr.stRcAttr.stH265Cbr.u32Gop = rk_param_get_int(entry_gop, -1);
			venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = rk_param_get_int(entry_max_rate, -1);
		} else {
			venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
			venc_chn_attr.stRcAttr.stH265Vbr.u32Gop = rk_param_get_int(entry_gop, -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32BitRate = rk_param_get_int(entry_mid_rate, -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32MaxBitRate = rk_param_get_int(entry_max_rate, -1);
			venc_chn_attr.stRcAttr.stH265Vbr.u32MinBitRate = rk_param_get_int(entry_min_rate, -1);
		}
	} else {
		LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
		return -1;
	}
	RK_MPI_VENC_SetChnAttr(stream_id, &venc_chn_attr);
	rk_param_set_string(entry_rc_mode, value);
	rk_video_reset_frame_rate(stream_id);

	return 0;
}

int rk_video_get_output_data_type(int stream_id, const char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:output_data_type", stream_id);
	*value = rk_param_get_string(entry, "H.265");

	return 0;
}

int rk_video_set_output_data_type(int stream_id, const char *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:output_data_type", stream_id);
	rk_param_set_string(entry, value);

	rk_video_restart();

	return 0;
}

int rk_video_get_rc_quality(int stream_id, const char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:rc_quality", stream_id);
	*value = rk_param_get_string(entry, "high");

	return 0;
}

int rk_video_set_rc_quality(int stream_id, const char *value) {
	char entry_rc_quality[128] = {'\0'};
	char entry_output_data_type[128] = {'\0'};

	snprintf(entry_rc_quality, 127, "video.%d:rc_quality", stream_id);
	snprintf(entry_output_data_type, 127, "video.%d:output_data_type", stream_id);
	tmp_output_data_type = rk_param_get_string(entry_output_data_type, "H.264");

	VENC_RC_PARAM_S venc_rc_param;
	RK_MPI_VENC_GetRcParam(stream_id, &venc_rc_param);
	if (!strcmp(tmp_output_data_type, "H.264")) {
		if (!strcmp(value, "highest")) {
			venc_rc_param.stParamH264.u32MinQp = 10;
		} else if (!strcmp(value, "higher")) {
			venc_rc_param.stParamH264.u32MinQp = 15;
		} else if (!strcmp(value, "high")) {
			venc_rc_param.stParamH264.u32MinQp = 20;
		} else if (!strcmp(value, "medium")) {
			venc_rc_param.stParamH264.u32MinQp = 25;
		} else if (!strcmp(value, "low")) {
			venc_rc_param.stParamH264.u32MinQp = 30;
		} else if (!strcmp(value, "lower")) {
			venc_rc_param.stParamH264.u32MinQp = 35;
		} else {
			venc_rc_param.stParamH264.u32MinQp = 40;
		}
	} else if (!strcmp(tmp_output_data_type, "H.265")) {
		if (!strcmp(value, "highest")) {
			venc_rc_param.stParamH265.u32MinQp = 10;
		} else if (!strcmp(value, "higher")) {
			venc_rc_param.stParamH265.u32MinQp = 15;
		} else if (!strcmp(value, "high")) {
			venc_rc_param.stParamH265.u32MinQp = 20;
		} else if (!strcmp(value, "medium")) {
			venc_rc_param.stParamH265.u32MinQp = 25;
		} else if (!strcmp(value, "low")) {
			venc_rc_param.stParamH265.u32MinQp = 30;
		} else if (!strcmp(value, "lower")) {
			venc_rc_param.stParamH265.u32MinQp = 35;
		} else {
			venc_rc_param.stParamH265.u32MinQp = 40;
		}
	} else {
		LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
		return -1;
	}
	RK_MPI_VENC_SetRcParam(stream_id, &venc_rc_param);
	rk_param_set_string(entry_rc_quality, value);

	return 0;
}

int rk_video_get_smart(int stream_id, const char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:smart", stream_id);
	*value = rk_param_get_string(entry, "close");

	return 0;
}

int rk_video_set_smart(int stream_id, const char *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:smart", stream_id);
	rk_param_set_string(entry, value);
	rk_video_restart();

	return 0;
}

int rk_video_get_gop_mode(int stream_id, const char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:gop_mode", stream_id);
	*value = rk_param_get_string(entry, "normalP");

	return 0;
}

int rk_video_set_gop_mode(int stream_id, const char *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:gop_mode", stream_id);
	rk_param_set_string(entry, value);
	rk_video_restart();

	return 0;
}
int rk_video_get_stream_type(int stream_id, const char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:stream_type", stream_id);
	*value = rk_param_get_string(entry, "mainStream");

	return 0;
}

int rk_video_set_stream_type(int stream_id, const char *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:stream_type", stream_id);
	rk_param_set_string(entry, value);

	return 0;
}

int rk_video_get_h264_profile(int stream_id, const char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:h264_profile", stream_id);
	*value = rk_param_get_string(entry, "high");

	return 0;
}

int rk_video_set_h264_profile(int stream_id, const char *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:h264_profile", stream_id);
	rk_param_set_string(entry, value);
	rk_video_restart();

	return 0;
}

int rk_video_get_resolution(int stream_id, char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:width", stream_id);
	int width = rk_param_get_int(entry, 0);
	snprintf(entry, 127, "video.%d:height", stream_id);
	int height = rk_param_get_int(entry, 0);
	sprintf(*value, "%d*%d", width, height);

	return 0;
}

int rk_video_set_resolution(int stream_id, const char *value) {
	char entry[128] = {'\0'};
	int width, height, ret;

	sscanf(value, "%d*%d", &width, &height);
	LOG_INFO("value is %s, width is %d, height is %d\n", value, width, height);

	// unbind
	vi_chn.enModId = RK_ID_VI;
	vi_chn.s32DevId = 0;
	vi_chn.s32ChnId = stream_id;
	venc_chn.enModId = RK_ID_VENC;
	venc_chn.s32DevId = 0;
	venc_chn.s32ChnId = stream_id;
	ret = RK_MPI_SYS_UnBind(&vi_chn, &venc_chn);
	if (ret)
		LOG_ERROR("Unbind VI and VENC error! ret=%#x\n", ret);
	else
		LOG_DEBUG("Unbind VI and VENC success\n");

	snprintf(entry, 127, "video.%d:width", stream_id);
	rk_param_set_int(entry, width);
	snprintf(entry, 127, "video.%d:height", stream_id);
	rk_param_set_int(entry, height);

	VENC_CHN_ATTR_S venc_chn_attr;
	RK_MPI_VENC_GetChnAttr(stream_id, &venc_chn_attr);
	venc_chn_attr.stVencAttr.u32PicWidth = width;
	venc_chn_attr.stVencAttr.u32PicHeight = height;
	venc_chn_attr.stVencAttr.u32VirWidth = width;
	venc_chn_attr.stVencAttr.u32VirHeight = height;
	ret = RK_MPI_VENC_SetChnAttr(stream_id, &venc_chn_attr);
	if (ret)
		LOG_ERROR("RK_MPI_VENC_SetChnAttr error! ret=%#x\n", ret);

	if (enable_jpeg && (stream_id == 0)) {
		snprintf(entry, 127, "video.jpeg:width");
		rk_param_set_int(entry, width);
		snprintf(entry, 127, "video.jpeg:height");
		rk_param_set_int(entry, height);

		RK_MPI_VENC_GetChnAttr(JPEG_VENC_CHN, &venc_chn_attr);
		venc_chn_attr.stVencAttr.u32PicWidth = width;
		venc_chn_attr.stVencAttr.u32PicHeight = height;
		venc_chn_attr.stVencAttr.u32VirWidth = width;
		venc_chn_attr.stVencAttr.u32VirHeight = height;
		ret = RK_MPI_VENC_SetChnAttr(JPEG_VENC_CHN, &venc_chn_attr);
		if (ret)
			LOG_ERROR("JPEG RK_MPI_VENC_SetChnAttr error! ret=%#x\n", ret);
	}
	VI_CHN_ATTR_S vi_chn_attr;
	RK_MPI_VI_GetChnAttr(0, stream_id, &vi_chn_attr);
	vi_chn_attr.stSize.u32Width = width;
	vi_chn_attr.stSize.u32Height = height;
	ret = RK_MPI_VI_SetChnAttr(pipe_id_, stream_id, &vi_chn_attr);
	if (ret)
		LOG_ERROR("RK_MPI_VI_SetChnAttr error! ret=%#x\n", ret);

	if (stream_id == DRAW_NN_VENC_CHN_ID && enable_npu)
		rkipc_osd_draw_nn_change();
	rk_osd_privacy_mask_restart();
	rk_roi_set_all(); // update roi info, and osd cover attach vi, no update required
	ret = RK_MPI_SYS_Bind(&vi_chn, &venc_chn);
	if (ret)
		LOG_ERROR("Unbind VI and VENC error! ret=%#x\n", ret);

	return 0;
}

int rk_video_get_frame_rate(int stream_id, char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:dst_frame_rate_den", stream_id);
	int den = rk_param_get_int(entry, -1);
	snprintf(entry, 127, "video.%d:dst_frame_rate_num", stream_id);
	int num = rk_param_get_int(entry, -1);
	if (den == 1)
		sprintf(*value, "%d", num);
	else
		sprintf(*value, "%d/%d", num, den);

	return 0;
}

int rk_video_set_frame_rate(int stream_id, const char *value) {
	char entry[128] = {'\0'};
	int den, num, sensor_fps;
	VI_CHN_ATTR_S vi_chn_attr;
	VENC_CHN_ATTR_S venc_chn_attr;

	if (strchr(value, '/') == NULL) {
		den = 1;
		sscanf(value, "%d", &num);
	} else {
		sscanf(value, "%d/%d", &num, &den);
	}
	LOG_INFO("num is %d, den is %d\n", num, den);
	sensor_fps = rk_param_get_int("isp.0.adjustment:fps", 30);

	RK_MPI_VI_GetChnAttr(pipe_id_, stream_id, &vi_chn_attr);
	LOG_INFO("old VI framerate is [%d:%d]\n", vi_chn_attr.stFrameRate.s32SrcFrameRate,
	         vi_chn_attr.stFrameRate.s32DstFrameRate);
	RK_MPI_VENC_GetChnAttr(stream_id, &venc_chn_attr);
	snprintf(entry, 127, "video.%d:output_data_type", stream_id);
	tmp_output_data_type = rk_param_get_string(entry, "H.264");
	snprintf(entry, 127, "video.%d:rc_mode", stream_id);
	tmp_rc_mode = rk_param_get_string(entry, "CBR");

	// if the frame rate is not an integer, use VENC frame rate control,
	// otherwise use VI frame rate control
	if (den != 1) {
		vi_chn_attr.stFrameRate.s32SrcFrameRate = sensor_fps;
		vi_chn_attr.stFrameRate.s32DstFrameRate = sensor_fps;

		if (!strcmp(tmp_output_data_type, "H.264")) {
			venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_AVC;
			if (!strcmp(tmp_rc_mode, "CBR")) {
				venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateDen = 1;
				venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateNum = sensor_fps;
				venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateNum = num;
			} else {
				venc_chn_attr.stRcAttr.stH264Vbr.u32SrcFrameRateDen = 1;
				venc_chn_attr.stRcAttr.stH264Vbr.u32SrcFrameRateNum = sensor_fps;
				venc_chn_attr.stRcAttr.stH264Vbr.fr32DstFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH264Vbr.fr32DstFrameRateNum = num;
			}
		} else if (!strcmp(tmp_output_data_type, "H.265")) {
			venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_HEVC;
			if (!strcmp(tmp_rc_mode, "CBR")) {
				venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
				venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateNum = sensor_fps;
				venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateNum = num;
			} else {
				venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateDen = 1;
				venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateNum = sensor_fps;
				venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateNum = num;
			}
		} else {
			LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
			return -1;
		}
	} else {
		vi_chn_attr.stFrameRate.s32SrcFrameRate = sensor_fps;
		vi_chn_attr.stFrameRate.s32DstFrameRate = num; // den == 1

		if (!strcmp(tmp_output_data_type, "H.264")) {
			venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_AVC;
			if (!strcmp(tmp_rc_mode, "CBR")) {
				venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateNum = num;
				venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateNum = num;
			} else {
				venc_chn_attr.stRcAttr.stH264Vbr.u32SrcFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH264Vbr.u32SrcFrameRateNum = num;
				venc_chn_attr.stRcAttr.stH264Vbr.fr32DstFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH264Vbr.fr32DstFrameRateNum = num;
			}
		} else if (!strcmp(tmp_output_data_type, "H.265")) {
			venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_HEVC;
			if (!strcmp(tmp_rc_mode, "CBR")) {
				venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateNum = num;
				venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateNum = num;
			} else {
				venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateNum = num;
				venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateDen = den;
				venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateNum = num;
			}
		} else {
			LOG_ERROR("tmp_output_data_type is %s, not support\n", tmp_output_data_type);
			return -1;
		}
	}
	LOG_INFO("new VI framerate is [%d:%d]\n", vi_chn_attr.stFrameRate.s32SrcFrameRate,
	         vi_chn_attr.stFrameRate.s32DstFrameRate);
	RK_MPI_VI_SetChnAttr(pipe_id_, stream_id, &vi_chn_attr);
	RK_MPI_VENC_SetChnAttr(stream_id, &venc_chn_attr);

	snprintf(entry, 127, "video.%d:dst_frame_rate_den", stream_id);
	rk_param_set_int(entry, den);
	snprintf(entry, 127, "video.%d:dst_frame_rate_num", stream_id);
	rk_param_set_int(entry, num);

	return 0;
}

int rk_video_reset_frame_rate(int stream_id) {
	int ret = 0;
	char *value = malloc(20);
	ret |= rk_video_get_frame_rate(stream_id, &value);
	ret |= rk_video_set_frame_rate(stream_id, value);
	free(value);

	return 0;
}

int rk_video_get_frame_rate_in(int stream_id, char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:src_frame_rate_den", stream_id);
	int den = rk_param_get_int(entry, -1);
	snprintf(entry, 127, "video.%d:src_frame_rate_num", stream_id);
	int num = rk_param_get_int(entry, -1);
	if (den == 1)
		sprintf(*value, "%d", num);
	else
		sprintf(*value, "%d/%d", num, den);

	return 0;
}

int rk_video_set_frame_rate_in(int stream_id, const char *value) {
	char entry[128] = {'\0'};
	int den, num;
	if (strchr(value, '/') == NULL) {
		den = 1;
		sscanf(value, "%d", &num);
	} else {
		sscanf(value, "%d/%d", &num, &den);
	}
	LOG_INFO("num is %d, den is %d\n", num, den);
	snprintf(entry, 127, "video.%d:src_frame_rate_den", stream_id);
	rk_param_set_int(entry, den);
	snprintf(entry, 127, "video.%d:src_frame_rate_num", stream_id);
	rk_param_set_int(entry, num);
	rk_video_restart();

	return 0;
}

int rk_video_get_rotation(int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.source:rotation");
	*value = rk_param_get_int(entry, 0);

	return 0;
}

int rk_video_set_rotation(int value) {
	int ret = 0;
	int rotation = 0;
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.source:rotation");

	rk_param_set_int(entry, value);
	if (value == 0) {
		rotation = ROTATION_0;
	} else if (value == 90) {
		rotation = ROTATION_90;
	} else if (value == 180) {
		rotation = ROTATION_180;
	} else if (value == 270) {
		rotation = ROTATION_270;
	}
	if (!enable_wrap) {
		ret = RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_0, rotation);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_SetChnRotation VIDEO_PIPE_0 error! ret=%#x\n", ret);
		ret = RK_MPI_VENC_SetChnRotation(JPEG_VENC_CHN, rotation);
		if (ret)
			LOG_ERROR("RK_MPI_VENC_SetChnRotation JPEG_VENC_CHN error! ret=%#x\n", ret);
	} else {
		LOG_WARN("enable wrap, venc-0 and jpeg can't rotate\n");
	}

	ret = RK_MPI_VENC_SetChnRotation(VIDEO_PIPE_1, rotation);
	if (ret)
		LOG_ERROR("RK_MPI_VENC_SetChnRotation VIDEO_PIPE_1 error! ret=%#x\n", ret);
	rk_roi_set_all(); // update roi info
	// update osd info, cover currently attaches to VI
	rk_osd_privacy_mask_restart();
	if (enable_npu)
		rkipc_osd_draw_nn_change();

	return 0;
}

int rk_video_get_smartp_viridrlen(int stream_id, int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.%d:smartp_viridrlen", stream_id);
	*value = rk_param_get_int(entry, 25);

	return 0;
}

int rk_video_set_smartp_viridrlen(int stream_id, int value) {
	char entry[128] = {'\0'};

	VENC_CHN_ATTR_S venc_chn_attr;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	RK_MPI_VENC_GetChnAttr(stream_id, &venc_chn_attr);
	venc_chn_attr.stGopAttr.s32VirIdrLen = value;
	RK_MPI_VENC_SetChnAttr(stream_id, &venc_chn_attr);

	snprintf(entry, 127, "video.%d:smartp_viridrlen", stream_id);
	rk_param_set_int(entry, value);

	return 0;
}

int rk_video_get_md_switch(int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "ivs:md");
	*value = rk_param_get_int(entry, -1);

	return 0;
}

int rk_video_set_md_switch(int value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "ivs:md");
	rk_param_set_int(entry, value);
	rk_video_restart();

	return 0;
}

int rk_video_get_md_sensebility(int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "ivs:md_sensibility");
	*value = rk_param_get_int(entry, -1);

	return 0;
}

int rk_video_set_md_sensebility(int value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "ivs:md_sensibility");
	rk_param_set_int(entry, value);
	rk_video_restart();
}

int rk_video_get_od_switch(int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "ivs:od");
	*value = rk_param_get_int(entry, -1);

	return 0;
}

int rk_video_set_od_switch(int value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "ivs:od");
	rk_param_set_int(entry, value);
	rk_video_restart();
}

int rkipc_osd_cover_create(int id, osd_data_s *osd_data) {
	LOG_DEBUG("id is %d\n", id);
	int ret = 0;
	RGN_HANDLE coverHandle = id;
	RGN_ATTR_S stCoverAttr;
	MPP_CHN_S stCoverChn;
	RGN_CHN_ATTR_S stCoverChnAttr;
	int rotation = rk_param_get_int("video.source:rotation", 0);
	int video_0_width = rk_param_get_int("video.0:width", -1);
	int video_0_height = rk_param_get_int("video.0:height", -1);
	int video_0_max_width = rk_param_get_int("video.0:max_width", -1);
	int video_0_max_height = rk_param_get_int("video.0:max_height", -1);
	double video_0_w_h_rate = 1.0;

	// since the coordinates stored in the OSD module are of actual resolution,
	// 1106 needs to be converted back to the maximum resolution
	osd_data->origin_x = osd_data->origin_x * video_0_max_width / video_0_width;
	osd_data->origin_y = osd_data->origin_y * video_0_max_height / video_0_height;
	osd_data->width = osd_data->width * video_0_max_width / video_0_width;
	osd_data->height = osd_data->height * video_0_max_height / video_0_height;

	memset(&stCoverAttr, 0, sizeof(stCoverAttr));
	memset(&stCoverChnAttr, 0, sizeof(stCoverChnAttr));
	// create cover regions
	stCoverAttr.enType = COVER_RGN;
	ret = RK_MPI_RGN_Create(coverHandle, &stCoverAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_Create (%d) failed with %#x\n", coverHandle, ret);
		RK_MPI_RGN_Destroy(coverHandle);
		return RK_FAILURE;
	}
	LOG_DEBUG("The handle: %d, create success\n", coverHandle);

	// when cover is attached to VI,
	// coordinate conversion of three angles shall be considered when rotating VENC
	video_0_w_h_rate = (double)video_0_max_width / (double)video_0_max_height;
	if (rotation == 90) {
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32X =
		    (double)osd_data->origin_y * video_0_w_h_rate;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32Y =
		    (video_0_max_height -
		     ((double)(osd_data->width + osd_data->origin_x) / video_0_w_h_rate));
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Width =
		    (double)osd_data->height * video_0_w_h_rate;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Height =
		    (double)osd_data->width / video_0_w_h_rate;
	} else if (rotation == 270) {
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32X =
		    (video_0_max_width -
		     ((double)(osd_data->height + osd_data->origin_y) * video_0_w_h_rate));
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32Y =
		    (double)osd_data->origin_x / video_0_w_h_rate;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Width =
		    (double)osd_data->height * video_0_w_h_rate;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Height =
		    (double)osd_data->width / video_0_w_h_rate;
	} else if (rotation == 180) {
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32X =
		    video_0_max_width - osd_data->width - osd_data->origin_x;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32Y =
		    video_0_max_height - osd_data->height - osd_data->origin_y;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Width = osd_data->width;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Height = osd_data->height;
	} else {
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32X = osd_data->origin_x;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32Y = osd_data->origin_y;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Width = osd_data->width;
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Height = osd_data->height;
	}
	stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32X =
	    UPALIGNTO16(stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32X);
	stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32Y =
	    UPALIGNTO16(stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32Y);
	stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Width =
	    UPALIGNTO16(stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Width);
	stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Height =
	    UPALIGNTO16(stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Height);
	// because the rotation is done in the VENC,
	// and the cover and VI resolution are both before the rotation,
	// there is no need to judge the rotation here
	while (stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32X +
	           stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Width >
	       video_0_max_width) {
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Width -= 16;
	}
	while (stCoverChnAttr.unChnAttr.stCoverChn.stRect.s32Y +
	           stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Height >
	       video_0_max_height) {
		stCoverChnAttr.unChnAttr.stCoverChn.stRect.u32Height -= 16;
	}

	// display cover regions to vi
	stCoverChn.enModId = RK_ID_VI;
	stCoverChn.s32DevId = 0;
	stCoverChn.s32ChnId = VI_MAX_CHN_NUM;
	stCoverChnAttr.bShow = osd_data->enable;
	stCoverChnAttr.enType = COVER_RGN;
	stCoverChnAttr.unChnAttr.stCoverChn.u32Color = 0xffffffff;
	stCoverChnAttr.unChnAttr.stCoverChn.u32Layer = id;
	LOG_DEBUG("cover region to chn success\n");
	ret = RK_MPI_RGN_AttachToChn(coverHandle, &stCoverChn, &stCoverChnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("vi pipe RK_MPI_RGN_AttachToChn (%d) failed with %#x\n", coverHandle, ret);
		return RK_FAILURE;
	}
	ret = RK_MPI_RGN_SetDisplayAttr(coverHandle, &stCoverChn, &stCoverChnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("vi pipe RK_MPI_RGN_SetDisplayAttr failed with %#x\n", ret);
		return RK_FAILURE;
	}
	LOG_DEBUG("RK_MPI_RGN_AttachToChn to vi 0 success\n");

	return ret;
}

int rkipc_osd_cover_destroy(int id) {
	LOG_DEBUG("%s\n", __func__);
	int ret = 0;
	// Detach osd from chn
	MPP_CHN_S stMppChn;
	RGN_HANDLE RgnHandle = id;

	stMppChn.enModId = RK_ID_VI;
	stMppChn.s32DevId = 0;
	stMppChn.s32ChnId = VI_MAX_CHN_NUM;
	ret = RK_MPI_RGN_DetachFromChn(RgnHandle, &stMppChn);
	if (RK_SUCCESS != ret)
		LOG_DEBUG("RK_MPI_RGN_DetachFrmChn (%d) to vi pipe failed with %#x\n", RgnHandle, ret);

	// destory region
	ret = RK_MPI_RGN_Destroy(RgnHandle);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_Destroy [%d] failed with %#x\n", RgnHandle, ret);
	}
	LOG_DEBUG("Destory handle:%d success\n", RgnHandle);

	return ret;
}

int rkipc_osd_bmp_create(int id, osd_data_s *osd_data) {
	LOG_DEBUG("id is %d\n", id);
	int ret = 0;
	RGN_HANDLE RgnHandle = id;
	RGN_ATTR_S stRgnAttr;
	MPP_CHN_S stMppChn;
	RGN_CHN_ATTR_S stRgnChnAttr;
	BITMAP_S stBitmap;

	// create overlay regions
	memset(&stRgnAttr, 0, sizeof(stRgnAttr));
	stRgnAttr.enType = OVERLAY_RGN;
	stRgnAttr.unAttr.stOverlay.enPixelFmt = RK_FMT_ARGB8888;
	stRgnAttr.unAttr.stOverlay.u32CanvasNum = 2;
	stRgnAttr.unAttr.stOverlay.stSize.u32Width = osd_data->width;
	stRgnAttr.unAttr.stOverlay.stSize.u32Height = osd_data->height;
	ret = RK_MPI_RGN_Create(RgnHandle, &stRgnAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_Create (%d) failed with %#x\n", RgnHandle, ret);
		RK_MPI_RGN_Destroy(RgnHandle);
		return RK_FAILURE;
	}
	LOG_DEBUG("The handle: %d, create success\n", RgnHandle);

	// display overlay regions to venc groups
	stMppChn.enModId = RK_ID_VENC;
	stMppChn.s32DevId = 0;
	stMppChn.s32ChnId = 0;
	memset(&stRgnChnAttr, 0, sizeof(stRgnChnAttr));
	stRgnChnAttr.bShow = osd_data->enable;
	stRgnChnAttr.enType = OVERLAY_RGN;
	stRgnChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = osd_data->origin_x;
	stRgnChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = osd_data->origin_y;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32BgAlpha = 128;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32FgAlpha = 128;
	stRgnChnAttr.unChnAttr.stOverlayChn.u32Layer = id;
	stMppChn.enModId = RK_ID_VENC;
	stMppChn.s32DevId = 0;

	// stRgnChnAttr.unChnAttr.stOverlayChn.stQpInfo.bEnable = true;
	// stRgnChnAttr.unChnAttr.stOverlayChn.stQpInfo.bForceIntra = false;
	// stRgnChnAttr.unChnAttr.stOverlayChn.stQpInfo.bAbsQp = false;
	// stRgnChnAttr.unChnAttr.stOverlayChn.stQpInfo.s32Qp = -3;
	if (enable_venc_0) {
		stMppChn.s32ChnId = 0;
		ret = RK_MPI_RGN_AttachToChn(RgnHandle, &stMppChn, &stRgnChnAttr);
		if (RK_SUCCESS != ret) {
			LOG_ERROR("RK_MPI_RGN_AttachToChn (%d) to venc0 failed with %#x\n", RgnHandle, ret);
			return RK_FAILURE;
		}
		LOG_DEBUG("RK_MPI_RGN_AttachToChn to venc0 success\n");
	}
	if (enable_venc_1) {
		stRgnChnAttr.unChnAttr.stOverlayChn.stPoint.s32X =
		    UPALIGNTO16(osd_data->origin_x * rk_param_get_int("video.1:width", 1) /
		                rk_param_get_int("video.0:width", 1));
		stRgnChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y =
		    UPALIGNTO16(osd_data->origin_y * rk_param_get_int("video.1:height", 1) /
		                rk_param_get_int("video.0:height", 1));
		stMppChn.s32ChnId = 1;
		ret = RK_MPI_RGN_AttachToChn(RgnHandle, &stMppChn, &stRgnChnAttr);
		if (RK_SUCCESS != ret) {
			LOG_ERROR("RK_MPI_RGN_AttachToChn (%d) to venc1 failed with %#x\n", RgnHandle, ret);
			return RK_FAILURE;
		}
		LOG_DEBUG("RK_MPI_RGN_AttachToChn to venc1 success\n");
	}
	if (enable_jpeg) {
		stMppChn.s32ChnId = JPEG_VENC_CHN;
		ret = RK_MPI_RGN_AttachToChn(RgnHandle, &stMppChn, &stRgnChnAttr);
		if (RK_SUCCESS != ret) {
			LOG_ERROR("RK_MPI_RGN_AttachToChn (%d) to jpeg failed with %#x\n", RgnHandle, ret);
			return RK_FAILURE;
		}
		LOG_DEBUG("RK_MPI_RGN_AttachToChn to jpeg success\n");
	}

	// set bitmap
	stBitmap.enPixelFormat = RK_FMT_ARGB8888;
	stBitmap.u32Width = osd_data->width;
	stBitmap.u32Height = osd_data->height;
	stBitmap.pData = (RK_VOID *)osd_data->buffer;
	ret = RK_MPI_RGN_SetBitMap(RgnHandle, &stBitmap);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("RK_MPI_RGN_SetBitMap failed with %#x\n", ret);
		return RK_FAILURE;
	}

	return ret;
}

int rkipc_osd_bmp_destroy(int id) {
	LOG_DEBUG("%s\n", __func__);
	int ret = 0;
	// Detach osd from chn
	MPP_CHN_S stMppChn;
	RGN_HANDLE RgnHandle = id;
	stMppChn.enModId = RK_ID_VENC;
	stMppChn.s32DevId = 0;
	if (enable_venc_0) {
		stMppChn.s32ChnId = 0;
		ret = RK_MPI_RGN_DetachFromChn(RgnHandle, &stMppChn);
		if (RK_SUCCESS != ret)
			LOG_DEBUG("RK_MPI_RGN_DetachFrmChn (%d) to venc0 failed with %#x\n", RgnHandle, ret);
	}
	if (enable_venc_1) {
		stMppChn.s32ChnId = 1;
		ret = RK_MPI_RGN_DetachFromChn(RgnHandle, &stMppChn);
		if (RK_SUCCESS != ret)
			LOG_DEBUG("RK_MPI_RGN_DetachFrmChn (%d) to venc1 failed with %#x\n", RgnHandle, ret);
	}
	if (enable_jpeg) {
		stMppChn.s32ChnId = JPEG_VENC_CHN;
		ret = RK_MPI_RGN_DetachFromChn(RgnHandle, &stMppChn);
		if (RK_SUCCESS != ret)
			LOG_DEBUG("RK_MPI_RGN_DetachFrmChn (%d) to jpeg failed with %#x\n", RgnHandle, ret);
	}

	// destory region
	ret = RK_MPI_RGN_Destroy(RgnHandle);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_RGN_Destroy [%d] failed with %#x\n", RgnHandle, ret);
	}
	LOG_DEBUG("Destory handle:%d success\n", RgnHandle);

	return ret;
}

int rkipc_osd_bmp_change(int id, osd_data_s *osd_data) {
	// LOG_DEBUG("id is %d\n", id);
	int ret = 0;
	RGN_HANDLE RgnHandle = id;
	BITMAP_S stBitmap;

	// set bitmap
	stBitmap.enPixelFormat = RK_FMT_ARGB8888;
	stBitmap.u32Width = osd_data->width;
	stBitmap.u32Height = osd_data->height;
	stBitmap.pData = (RK_VOID *)osd_data->buffer;
	ret = RK_MPI_RGN_SetBitMap(RgnHandle, &stBitmap);
	if (ret != RK_SUCCESS) {
		LOG_ERROR("RK_MPI_RGN_SetBitMap failed with %#x\n", ret);
		return RK_FAILURE;
	}

	return ret;
}

int rkipc_osd_init() {
	rk_osd_cover_create_callback_register(rkipc_osd_cover_create);
	rk_osd_cover_destroy_callback_register(rkipc_osd_cover_destroy);
	rk_osd_bmp_create_callback_register(rkipc_osd_bmp_create);
	rk_osd_bmp_destroy_callback_register(rkipc_osd_bmp_destroy);
	rk_osd_bmp_change_callback_register(rkipc_osd_bmp_change);
	rk_osd_init();

	return 0;
}

int rkipc_osd_deinit() {
	rk_osd_deinit();
	rk_osd_cover_create_callback_register(NULL);
	rk_osd_cover_destroy_callback_register(NULL);
	rk_osd_bmp_create_callback_register(NULL);
	rk_osd_bmp_destroy_callback_register(NULL);
	rk_osd_bmp_change_callback_register(NULL);

	return 0;
}

// jpeg
int rk_video_get_enable_cycle_snapshot(int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.jpeg:enable_cycle_snapshot");
	*value = rk_param_get_int(entry, -1);

	return 0;
}

int rk_video_set_enable_cycle_snapshot(int value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.jpeg:enable_cycle_snapshot");
	rk_param_set_int(entry, value);
	if (value && !cycle_snapshot_flag) {
		cycle_snapshot_flag = 1;
		pthread_create(&cycle_snapshot_thread_id, NULL, rkipc_cycle_snapshot, NULL);
	} else if (!value && cycle_snapshot_flag) {
		get_jpeg_cnt = 0;
		cycle_snapshot_flag = 0;
		pthread_join(cycle_snapshot_thread_id, NULL);
	}

	return 0;
}

int rk_video_get_image_quality(int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.jpeg:jpeg_qfactor");
	*value = rk_param_get_int(entry, -1);

	return 0;
}

int rk_video_set_image_quality(int value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.jpeg:jpeg_qfactor");
	rk_param_set_int(entry, value);

	VENC_JPEG_PARAM_S stJpegParam;
	memset(&stJpegParam, 0, sizeof(stJpegParam));
	stJpegParam.u32Qfactor = value;
	RK_MPI_VENC_SetJpegParam(JPEG_VENC_CHN, &stJpegParam);

	return 0;
}

int rk_video_get_snapshot_interval_ms(int *value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.jpeg:snapshot_interval_ms");
	*value = rk_param_get_int(entry, 0);

	return 0;
}

int rk_video_set_snapshot_interval_ms(int value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.jpeg:snapshot_interval_ms");
	rk_param_set_int(entry, value);

	return 0;
}

int rk_video_get_jpeg_resolution(char **value) {
	char entry[128] = {'\0'};
	snprintf(entry, 127, "video.jpeg:width");
	int width = rk_param_get_int(entry, 0);
	snprintf(entry, 127, "video.jpeg:height");
	int height = rk_param_get_int(entry, 0);
	sprintf(*value, "%d*%d", width, height);

	return 0;
}

int rk_video_set_jpeg_resolution(const char *value) {
#if 0
	int width, height, ret;
	char entry[128] = {'\0'};
	sscanf(value, "%d*%d", &width, &height);
	snprintf(entry, 127, "video.jpeg:width");
	rk_param_set_int(entry, width);
	snprintf(entry, 127, "video.jpeg:height");
	rk_param_set_int(entry, height);

	VENC_CHN_ATTR_S venc_chn_attr;
	RK_MPI_VENC_GetChnAttr(JPEG_VENC_CHN, &venc_chn_attr);
	venc_chn_attr.stVencAttr.u32PicWidth = width;
	venc_chn_attr.stVencAttr.u32PicHeight = height;
	venc_chn_attr.stVencAttr.u32VirWidth = width;
	venc_chn_attr.stVencAttr.u32VirHeight = height;
	ret = RK_MPI_VENC_SetChnAttr(JPEG_VENC_CHN, &venc_chn_attr);
	if (ret)
		LOG_ERROR("JPEG RK_MPI_VENC_SetChnAttr error! ret=%#x\n", ret);
#else
	LOG_INFO("1103 combo, jpeg resolution must be consistent with the main stream resolution\n");
#endif

	return 0;
}

int rk_take_photo() {
	LOG_DEBUG("start\n");
	if (get_jpeg_cnt) {
		LOG_WARN("the last photo was not completed\n");
		return -1;
	}
	if (rkipc_storage_dev_mount_status_get() != DISK_MOUNTED) {
		LOG_WARN("dev not mount\n");
		return -1;
	}
	VENC_RECV_PIC_PARAM_S stRecvParam;
	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = 1;
	RK_MPI_VENC_StartRecvFrame(JPEG_VENC_CHN, &stRecvParam);
	get_jpeg_cnt++;

	return 0;
}

int rk_roi_set(roi_data_s *roi_data) {
	// LOG_DEBUG("id is %d\n", id);
	int ret = 0;
	int venc_chn = -1;
	VENC_ROI_ATTR_S pstRoiAttr;
	pstRoiAttr.u32Index = roi_data->id;
	pstRoiAttr.bEnable = roi_data->enabled;
	pstRoiAttr.bAbsQp = RK_FALSE;
	pstRoiAttr.bIntra = RK_FALSE;
	pstRoiAttr.stRect.s32X = roi_data->position_x;
	pstRoiAttr.stRect.s32Y = roi_data->position_y;
	pstRoiAttr.stRect.u32Width = roi_data->width;
	pstRoiAttr.stRect.u32Height = roi_data->height;
	switch (roi_data->quality_level) {
	case 6:
		pstRoiAttr.s32Qp = -16;
		break;
	case 5:
		pstRoiAttr.s32Qp = -14;
		break;
	case 4:
		pstRoiAttr.s32Qp = -12;
		break;
	case 3:
		pstRoiAttr.s32Qp = -10;
		break;
	case 2:
		pstRoiAttr.s32Qp = -8;
		break;
	case 1:
	default:
		pstRoiAttr.s32Qp = -6;
	}

	if (!strcmp(roi_data->stream_type, "mainStream") &&
	    rk_param_get_int("video.source:enable_venc_0", 0)) {
		venc_chn = 0;
	} else if (!strcmp(roi_data->stream_type, "subStream") &&
	           rk_param_get_int("video.source:enable_venc_1", 0)) {
		venc_chn = 1;
	} else if (!strcmp(roi_data->stream_type, "thirdStream") &&
	           rk_param_get_int("video.source:enable_venc_2", 0)) {
		venc_chn = 2;
	} else {
		LOG_DEBUG("%s is not exit\n", roi_data->stream_type);
		return -1;
	}

	ret = RK_MPI_VENC_SetRoiAttr(venc_chn, &pstRoiAttr);
	if (RK_SUCCESS != ret) {
		LOG_ERROR("RK_MPI_VENC_SetRoiAttr to venc %d failed with %#x\n", venc_chn, ret);
		return RK_FAILURE;
	}
	LOG_DEBUG("RK_MPI_VENC_SetRoiAttr to venc %d success\n", venc_chn);

	return ret;
}

// int rk_region_clip_set(int venc_chn, region_clip_data_s *region_clip_data) {
// 	int ret = 0;
// 	VENC_CHN_PARAM_S stParam;

// 	RK_MPI_VENC_GetChnParam(venc_chn, &stParam);
// 	if (RK_SUCCESS != ret) {
// 		LOG_ERROR("RK_MPI_VENC_GetChnParam to venc failed with %#x\n", ret);
// 		return RK_FAILURE;
// 	}
// 	LOG_DEBUG("RK_MPI_VENC_GetChnParam to venc success\n");
// 	LOG_DEBUG("venc_chn is %d\n", venc_chn);
// 	if (region_clip_data->enabled)
// 		stParam.stCropCfg.enCropType = VENC_CROP_ONLY;
// 	else
// 		stParam.stCropCfg.enCropType = VENC_CROP_NONE;
// 	stParam.stCropCfg.stCropRect.s32X = region_clip_data->position_x;
// 	stParam.stCropCfg.stCropRect.s32Y = region_clip_data->position_y;
// 	stParam.stCropCfg.stCropRect.u32Width = region_clip_data->width;
// 	stParam.stCropCfg.stCropRect.u32Height = region_clip_data->height;
// 	LOG_DEBUG("xywh is %d,%d,%d,%d\n", stParam.stCropCfg.stCropRect.s32X,
// stParam.stCropCfg.stCropRect.s32Y, 				stParam.stCropCfg.stCropRect.u32Width,
// stParam.stCropCfg.stCropRect.u32Height); 	ret = RK_MPI_VENC_SetChnParam(venc_chn, &stParam);
// if
// (RK_SUCCESS != ret) { 		LOG_ERROR("RK_MPI_VENC_SetChnParam to venc failed with %#x\n", ret);
// return RK_FAILURE;
// 	}
// 	LOG_DEBUG("RK_MPI_VENC_SetChnParam to venc success\n");

// 	return ret;
// }

// ========== USB Camera Module (V4L2 + VDEC + VENC) ==========

// Find USB camera device that supports MJPG at desired resolution
static int usb_camera_find_device(int *fd_out, int *out_width, int *out_height, int *out_fps) {
	char dev_path[32];

	// Resolution fallback chain: requested -> 1280x720 -> 640x480 -> 320x240
	struct { int w, h; } res_table[] = {
		{ *out_width, *out_height },
		{ 1280, 720 },
		{ 640, 480 },
		{ 320, 240 },
	};
	int res_count = sizeof(res_table) / sizeof(res_table[0]);

	for (int i = 0; i < 10; i++) {
		snprintf(dev_path, sizeof(dev_path), "/dev/video%d", i);
		int fd = open(dev_path, O_RDWR);
		if (fd < 0)
			continue;

		struct v4l2_capability cap;
		if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0 ||
		    !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
		    !(cap.capabilities & V4L2_CAP_STREAMING)) {
			close(fd);
			continue;
		}

		// Check for MJPEG support
		int has_mjpeg = 0;
		struct v4l2_fmtdesc fmtdesc;
		memset(&fmtdesc, 0, sizeof(fmtdesc));
		fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
			if (fmtdesc.pixelformat == V4L2_PIX_FMT_MJPEG) {
				has_mjpeg = 1;
				break;
			}
			fmtdesc.index++;
		}
		if (!has_mjpeg) {
			close(fd);
			continue;
		}

		// Try each resolution in fallback chain
		int found_res = -1;
		for (int r = 0; r < res_count; r++) {
			int try_w = res_table[r].w;
			int try_h = res_table[r].h;

			struct v4l2_frmsizeenum frmsize;
			memset(&frmsize, 0, sizeof(frmsize));
			frmsize.pixel_format = V4L2_PIX_FMT_MJPEG;
			int has_res = 0;
			while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
				if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE &&
				    frmsize.discrete.width == (unsigned)try_w &&
				    frmsize.discrete.height == (unsigned)try_h) {
					has_res = 1;
					break;
				}
				frmsize.index++;
			}
			if (!has_res)
				continue;

			// Check desired fps, fallback to 15fps
			int actual_fps = 0;
			int try_fps[] = { *out_fps, 15 };
			for (int f = 0; f < 2; f++) {
				struct v4l2_frmivalenum frmival;
				memset(&frmival, 0, sizeof(frmival));
				frmival.pixel_format = V4L2_PIX_FMT_MJPEG;
				frmival.width = try_w;
				frmival.height = try_h;
				while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
					if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE &&
					    frmival.discrete.numerator == 1 &&
					    frmival.discrete.denominator == (unsigned)try_fps[f]) {
						actual_fps = try_fps[f];
						break;
					}
					frmival.index++;
				}
				if (actual_fps)
					break;
			}
			if (!actual_fps)
				continue;

			// Found a working resolution + fps combo
			found_res = r;
			*out_width = try_w;
			*out_height = try_h;
			*out_fps = actual_fps;
			break;
		}

		if (found_res < 0) {
			LOG_WARN("USB camera %s: no supported MJPG resolution found\n", dev_path);
			close(fd);
			continue;
		}

		if (found_res > 0)
			LOG_WARN("USB camera: requested resolution not supported, using fallback %dx%d @%dfps\n",
			         *out_width, *out_height, *out_fps);

		*fd_out = fd;
		LOG_INFO("Found USB camera: %s (MJPG %dx%d @%dfps)\n",
		         dev_path, *out_width, *out_height, *out_fps);
		return 0;
	}
	return -1;
}

static int usb_camera_init(void) {
	int w = rk_param_get_int("video.usb:width", USB_CAMERA_DEFAULT_WIDTH);
	int h = rk_param_get_int("video.usb:height", USB_CAMERA_DEFAULT_HEIGHT);
	int fps = rk_param_get_int("video.usb:fps", USB_CAMERA_DEFAULT_FPS);
	usb_cam_width_ = w;
	usb_cam_height_ = h;
	usb_cam_fps_ = fps;

	// Check if user specified a device path
	const char *dev_cfg = rk_param_get_string("video.usb:device", NULL);
	if (dev_cfg && strlen(dev_cfg) > 0) {
		usb_camera_fd_ = open(dev_cfg, O_RDWR);
		if (usb_camera_fd_ < 0) {
			LOG_WARN("USB camera: cannot open %s: %s\n", dev_cfg, strerror(errno));
			return -1;
		}
	} else {
		if (usb_camera_find_device(&usb_camera_fd_, &usb_cam_width_, &usb_cam_height_, &usb_cam_fps_) < 0) {
			LOG_WARN("USB camera: no suitable MJPG camera found\n");
			return -1;
		}
	}

	// Set format
	struct v4l2_format fmt;
	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = usb_cam_width_;
	fmt.fmt.pix.height = usb_cam_height_;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;
	if (ioctl(usb_camera_fd_, VIDIOC_S_FMT, &fmt) < 0) {
		LOG_ERROR("USB camera: VIDIOC_S_FMT failed: %s\n", strerror(errno));
		close(usb_camera_fd_);
		usb_camera_fd_ = -1;
		return -1;
	}

	// Set frame rate
	struct v4l2_streamparm parm;
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = usb_cam_fps_;
	ioctl(usb_camera_fd_, VIDIOC_S_PARM, &parm);

	// Request mmap buffers
	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof(req));
	req.count = USB_CAMERA_BUF_COUNT;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (ioctl(usb_camera_fd_, VIDIOC_REQBUFS, &req) < 0) {
		LOG_ERROR("USB camera: VIDIOC_REQBUFS failed: %s\n", strerror(errno));
		close(usb_camera_fd_);
		usb_camera_fd_ = -1;
		return -1;
	}

	// Map buffers
	for (unsigned i = 0; i < req.count; i++) {
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (ioctl(usb_camera_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
			LOG_ERROR("USB camera: VIDIOC_QUERYBUF %d failed\n", i);
			close(usb_camera_fd_);
			usb_camera_fd_ = -1;
			return -1;
		}
		usb_v4l2_buffers_[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
		                              MAP_SHARED, usb_camera_fd_, buf.m.offset);
		usb_v4l2_buffer_lengths_[i] = buf.length;
		// Queue buffer
		if (ioctl(usb_camera_fd_, VIDIOC_QBUF, &buf) < 0) {
			LOG_ERROR("USB camera: VIDIOC_QBUF %d failed\n", i);
		}
	}

	usb_camera_available_ = 1;
	LOG_INFO("USB camera initialized: MJPG %dx%d @%dfps\n",
	         usb_cam_width_, usb_cam_height_, usb_cam_fps_);
	return 0;
}

static void usb_camera_start_stream(void) {
	if (usb_camera_fd_ < 0 || usb_camera_streaming_)
		return;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(usb_camera_fd_, VIDIOC_STREAMON, &type) < 0) {
		LOG_ERROR("USB camera: VIDIOC_STREAMON failed: %s\n", strerror(errno));
		return;
	}
	usb_camera_streaming_ = 1;
	LOG_INFO("USB camera: stream started\n");
}

static void usb_camera_stop_stream(void) {
	if (usb_camera_fd_ < 0 || !usb_camera_streaming_)
		return;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(usb_camera_fd_, VIDIOC_STREAMOFF, &type);
	usb_camera_streaming_ = 0;
	LOG_INFO("USB camera: stream stopped\n");
}

static void usb_camera_deinit(void) {
	usb_camera_stop_stream();
	if (usb_camera_fd_ >= 0) {
		for (int i = 0; i < USB_CAMERA_BUF_COUNT; i++) {
			if (usb_v4l2_buffers_[i] && usb_v4l2_buffers_[i] != MAP_FAILED) {
				munmap(usb_v4l2_buffers_[i], usb_v4l2_buffer_lengths_[i]);
				usb_v4l2_buffers_[i] = NULL;
			}
		}
		close(usb_camera_fd_);
		usb_camera_fd_ = -1;
	}
	usb_camera_available_ = 0;
}

// VDEC JPEG decoder for USB camera MJPG frames
static int vdec_jpeg_init(void) {
	VDEC_CHN_ATTR_S attr;
	memset(&attr, 0, sizeof(attr));
	attr.enType = RK_VIDEO_ID_JPEG;
	attr.enMode = VIDEO_MODE_FRAME;
	attr.u32PicWidth = usb_cam_width_;
	attr.u32PicHeight = usb_cam_height_;
	attr.u32PicVirWidth = usb_cam_width_;
	attr.u32PicVirHeight = usb_cam_height_;
	attr.u32StreamBufSize = usb_cam_width_ * usb_cam_height_ * 3 / 2;
	attr.u32FrameBufSize = usb_cam_width_ * usb_cam_height_ * 3 / 2;
	attr.u32FrameBufCnt = 2;
	attr.u32StreamBufCnt = 8;

	int ret = RK_MPI_VDEC_CreateChn(VDEC_CHN_ID, &attr);
	if (ret) {
		LOG_ERROR("VDEC: CreateChn failed ret=%#x (VDEC may not be supported on this firmware)\n", ret);
		return ret;
	}

	// Set output pixel format to NV12
	VDEC_CHN_PARAM_S param;
	memset(&param, 0, sizeof(param));
	param.enType = RK_VIDEO_ID_JPEG;
	param.stVdecPictureParam.enPixelFormat = RK_FMT_YUV420SP;
	ret = RK_MPI_VDEC_SetChnParam(VDEC_CHN_ID, &param);
	if (ret)
		LOG_WARN("VDEC: SetChnParam ret=%#x\n", ret);

	ret = RK_MPI_VDEC_StartRecvStream(VDEC_CHN_ID);
	if (ret) {
		LOG_ERROR("VDEC: StartRecvStream failed ret=%#x\n", ret);
		RK_MPI_VDEC_DestroyChn(VDEC_CHN_ID);
		return ret;
	}

	LOG_INFO("VDEC JPEG decoder initialized (%dx%d)\n", usb_cam_width_, usb_cam_height_);
	return 0;
}

static void vdec_jpeg_deinit(void) {
	RK_MPI_VDEC_StopRecvStream(VDEC_CHN_ID);
	RK_MPI_VDEC_DestroyChn(VDEC_CHN_ID);
}

// Decode one MJPG frame to NV12 YUV via VDEC
static int vdec_decode_jpeg(void *jpeg_data, int jpeg_len, VIDEO_FRAME_INFO_S *out_frame) {
	VDEC_STREAM_S stream;
	memset(&stream, 0, sizeof(stream));
	stream.u32Len = jpeg_len;
	stream.bEndOfStream = RK_FALSE;
	stream.bEndOfFrame = RK_TRUE;

	// Allocate MB_BLK for JPEG data
	MB_BLK mbBlk = NULL;
	int ret = RK_MPI_SYS_MmzAlloc(&mbBlk, NULL, "vdec_jpeg", jpeg_len);
	if (ret) {
		LOG_ERROR("VDEC: MmzAlloc failed\n");
		return -1;
	}
	void *vir = RK_MPI_MB_Handle2VirAddr(mbBlk);
	memcpy(vir, jpeg_data, jpeg_len);
	RK_MPI_SYS_MmzFlushCache(mbBlk, RK_TRUE);
	stream.pMbBlk = mbBlk;

	ret = RK_MPI_VDEC_SendStream(VDEC_CHN_ID, &stream, 1000);
	RK_MPI_SYS_MmzFree(mbBlk);
	if (ret) {
		LOG_ERROR("VDEC: SendStream failed ret=%#x\n", ret);
		return -1;
	}

	ret = RK_MPI_VDEC_GetFrame(VDEC_CHN_ID, out_frame, 1000);
	if (ret) {
		LOG_ERROR("VDEC: GetFrame failed ret=%#x\n", ret);
		return -1;
	}
	return 0;
}

// Create VENC channel for USB camera H265 encoding
static int usb_venc_create(void) {
	VENC_CHN_ATTR_S attr;
	memset(&attr, 0, sizeof(attr));
	attr.stVencAttr.enType = RK_VIDEO_ID_HEVC;
	attr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	attr.stVencAttr.u32MaxPicWidth = usb_cam_width_;
	attr.stVencAttr.u32MaxPicHeight = usb_cam_height_;
	attr.stVencAttr.u32PicWidth = usb_cam_width_;
	attr.stVencAttr.u32PicHeight = usb_cam_height_;
	attr.stVencAttr.u32VirWidth = usb_cam_width_;
	attr.stVencAttr.u32VirHeight = usb_cam_height_;
	attr.stVencAttr.u32StreamBufCnt = 3;
	attr.stVencAttr.u32BufSize = usb_cam_width_ * usb_cam_height_ * 3 / 2;
	attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
	attr.stRcAttr.stH265Cbr.u32Gop = rk_param_get_int("video.usb:gop", 30);
	attr.stRcAttr.stH265Cbr.u32BitRate = rk_param_get_int("video.usb:max_rate", 1024);

	int ret = RK_MPI_VENC_CreateChn(USB_VENC_CHN, &attr);
	if (ret) {
		LOG_ERROR("USB VENC: CreateChn failed ret=%#x\n", ret);
		return ret;
	}

	VENC_RECV_PIC_PARAM_S stRecvParam;
	memset(&stRecvParam, 0, sizeof(stRecvParam));
	stRecvParam.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(USB_VENC_CHN, &stRecvParam);

	LOG_INFO("USB VENC channel %d created (H265 %dx%d)\n",
	         USB_VENC_CHN, usb_cam_width_, usb_cam_height_);
	return 0;
}

static void usb_venc_destroy(void) {
	RK_MPI_VENC_StopRecvFrame(USB_VENC_CHN);
	RK_MPI_VENC_DestroyChn(USB_VENC_CHN);
}

// USB camera encode thread with auto-reconnect:
//   Outer loop: detect device -> init -> encode -> on failure: cleanup -> re-detect
static void *usb_camera_encode_thread(void *arg) {
	(void)arg;
	prctl(PR_SET_NAME, "UsbCamEnc", 0, 0, 0);
	LOG_INFO("USB camera thread started (auto-reconnect enabled)\n");

	VENC_STREAM_S stFrame;
	stFrame.pstPack = malloc(sizeof(VENC_PACK_S));
	int ret;
	int vdec_created = 0;
	int venc_created = 0;
	int consecutive_errors = 0;

	while (g_video_run_) {
		// === Phase 1: Device discovery + V4L2 init (usb_camera_init handles both) ===
		if (usb_camera_init() != 0) {
			if (g_video_run_)
				LOG_INFO("USB camera: no device found, retrying in 3s...\n");
			for (int i = 0; i < 30 && g_video_run_; i++)
				usleep(100000);
			continue;
		}

		LOG_INFO("USB camera: device ready (%dx%d @%dfps), waiting for TCP client...\n",
		         usb_cam_width_, usb_cam_height_, usb_cam_fps_);
		consecutive_errors = 0;
		vdec_created = 0;
		venc_created = 0;

		// === Phase 2: Encode loop (runs until disconnect or exit) ===
		while (g_video_run_) {
			int client_connected = (__atomic_load_n(&tcp_video_client_fd[2], __ATOMIC_ACQUIRE) >= 0);

			if (!client_connected) {
				// No TCP client: stop streaming to save USB bandwidth for 4G module
				if (usb_camera_streaming_)
					usb_camera_stop_stream();
				if (venc_created) {
					usb_venc_destroy();
					venc_created = 0;
				}
				if (vdec_created) {
					vdec_jpeg_deinit();
					vdec_created = 0;
				}
				usleep(50000);
				continue;
			}

			// TCP client connected: start pipeline if not already running
			if (!vdec_created) {
				if (vdec_jpeg_init() != 0) {
					LOG_ERROR("USB camera: VDEC init failed, retrying...\n");
					usleep(500000);
					continue;
				}
				vdec_created = 1;
			}
			if (!venc_created) {
				if (usb_venc_create() != 0) {
					LOG_ERROR("USB camera: VENC init failed, retrying...\n");
					usleep(500000);
					continue;
				}
				venc_created = 1;
			}
			if (!usb_camera_streaming_)
				usb_camera_start_stream();

			// 1. Dequeue V4L2 buffer (get MJPG frame)
			struct v4l2_buffer vbuf;
			memset(&vbuf, 0, sizeof(vbuf));
			vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			vbuf.memory = V4L2_MEMORY_MMAP;
			ret = ioctl(usb_camera_fd_, VIDIOC_DQBUF, &vbuf);
			if (ret < 0) {
				if (errno == EAGAIN) {
					usleep(1000);
					continue;
				}
				LOG_ERROR("USB camera: DQBUF failed: %s (device disconnected?)\n", strerror(errno));
				consecutive_errors++;
				break;  // -> cleanup and reconnect
			}

			void *jpeg_data = usb_v4l2_buffers_[vbuf.index];
			int jpeg_len = vbuf.bytesused;

			// 2. Decode MJPG -> NV12 via VDEC
			VIDEO_FRAME_INFO_S dec_frame;
			if (vdec_decode_jpeg(jpeg_data, jpeg_len, &dec_frame) == 0) {
				// 3. Send decoded YUV frame to VENC
				VIDEO_FRAME_INFO_S vi_frame;
				memset(&vi_frame, 0, sizeof(vi_frame));
				vi_frame.stVFrame.pMbBlk = dec_frame.stVFrame.pMbBlk;
				vi_frame.stVFrame.u32Width = dec_frame.stVFrame.u32Width;
				vi_frame.stVFrame.u32Height = dec_frame.stVFrame.u32Height;
				vi_frame.stVFrame.u32VirWidth = dec_frame.stVFrame.u32VirWidth;
				vi_frame.stVFrame.u32VirHeight = dec_frame.stVFrame.u32VirHeight;
				vi_frame.stVFrame.enPixelFormat = dec_frame.stVFrame.enPixelFormat;
				vi_frame.stVFrame.u64PTS = dec_frame.stVFrame.u64PTS;

				ret = RK_MPI_VENC_SendFrame(USB_VENC_CHN, &vi_frame, 1000);
				RK_MPI_VDEC_ReleaseFrame(VDEC_CHN_ID, &dec_frame);

				if (ret == RK_SUCCESS) {
					// 4. Get H265 encoded frame
					ret = RK_MPI_VENC_GetStream(USB_VENC_CHN, &stFrame, 1000);
					if (ret == RK_SUCCESS) {
						void *data = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
						// 5. Send to TCP port 5105
						tcp_video_send_frame(2, data, stFrame.pstPack->u32Len);
						RK_MPI_VENC_ReleaseStream(USB_VENC_CHN, &stFrame);
					}
				}
			}

			// 6. Requeue V4L2 buffer
			if (ioctl(usb_camera_fd_, VIDIOC_QBUF, &vbuf) < 0) {
				LOG_ERROR("USB camera: QBUF failed: %s (device disconnected?)\n", strerror(errno));
				consecutive_errors++;
				break;  // -> cleanup and reconnect
			}

			consecutive_errors = 0;  // Reset on success
		}

		// === Phase 3: Cleanup after disconnect ===
		LOG_INFO("USB camera: cleaning up after disconnect (errors=%d)...\n", consecutive_errors);
		if (venc_created) {
			usb_venc_destroy();
			venc_created = 0;
		}
		if (vdec_created) {
			vdec_jpeg_deinit();
			vdec_created = 0;
		}
		usb_camera_deinit();

		// Wait before re-detecting device
		if (g_video_run_) {
			LOG_INFO("USB camera: will re-detect device in 3s...\n");
			for (int i = 0; i < 30 && g_video_run_; i++)
				usleep(100000);
		}
	}

	// Final cleanup
	if (stFrame.pstPack)
		free(stFrame.pstPack);

	LOG_INFO("USB camera thread exited\n");
	return NULL;
}

// ========== End USB Camera Module ==========

int rk_video_init() {
	LOG_DEBUG("begin\n");
	int ret = 0;
	enable_ivs = rk_param_get_int("video.source:enable_ivs", 1);
	enable_jpeg = rk_param_get_int("video.source:enable_jpeg", 1);
	enable_venc_0 = rk_param_get_int("video.source:enable_venc_0", 1);
	enable_venc_1 = rk_param_get_int("video.source:enable_venc_1", 1);
	enable_rtsp = rk_param_get_int("video.source:enable_rtsp", 1);
	enable_rtmp = rk_param_get_int("video.source:enable_rtmp", 1);
	enable_tcp_video = rk_param_get_int("video.source:enable_tcp_video", 1);
	LOG_INFO("enable_jpeg is %d, enable_venc_0 is %d, enable_venc_1 is %d, enable_rtsp is %d, "
	         "enable_rtmp is %d, enable_tcp_video is %d\n",
	         enable_jpeg, enable_venc_0, enable_venc_1, enable_rtsp, enable_rtmp, enable_tcp_video);

	g_vi_chn_id = rk_param_get_int("video.source:vi_chn_id", 0);
	g_enable_vo = rk_param_get_int("video.source:enable_vo", 1);
	g_vo_dev_id = rk_param_get_int("video.source:vo_dev_id", 3);
	enable_npu = rk_param_get_int("video.source:enable_npu", 0);
	enable_wrap = rk_param_get_int("video.source:enable_wrap", 0);
	enable_osd = rk_param_get_int("osd.common:enable_osd", 0);
	LOG_DEBUG("g_vi_chn_id is %d, g_enable_vo is %d, g_vo_dev_id is %d, enable_npu is %d, "
	          "enable_wrap is %d, enable_osd is %d\n",
	          g_vi_chn_id, g_enable_vo, g_vo_dev_id, enable_npu, enable_wrap, enable_osd);
	g_video_run_ = 1;
	ret |= rkipc_vi_dev_init();
	if (enable_rtsp)
		ret |= rkipc_rtsp_init(RTSP_URL_0, RTSP_URL_1, NULL);
	if (enable_rtmp)
		ret |= rkipc_rtmp_init();
	if (enable_tcp_video)
		ret |= tcp_video_server_init();
	ret |= tcp_control_server_init();
	if (enable_venc_0)
		ret |= rkipc_pipe_0_init();
	if (enable_venc_1)
		ret |= rkipc_pipe_1_init();
	if (enable_jpeg)
		ret |= rkipc_pipe_jpeg_init();
	// if (g_enable_vo)
	// 	ret |= rkipc_pipe_vpss_vo_init();
	rk_roi_set_callback_register(rk_roi_set);
	ret |= rk_roi_set_all();
	// rk_region_clip_set_callback_register(rk_region_clip_set);
	// rk_region_clip_set_all();
	if (enable_npu || enable_ivs) {
		ret |= rkipc_pipe_2_init();
	}
	// The osd dma buffer must be placed in the last application,
	// otherwise, when the font size is switched, holes may be caused
	if (enable_osd)
		ret |= rkipc_osd_init();

	// USB Camera (optional, auto-reconnect thread handles device discovery)
	if (rk_param_get_int("video.usb:enable", 1)) {
		pthread_create(&usb_camera_thread_, NULL, usb_camera_encode_thread, NULL);
		LOG_INFO("USB camera auto-reconnect thread created\n");
	}

	LOG_DEBUG("over\n");

	return ret;
}

int rk_video_deinit() {
	LOG_DEBUG("%s\n", __func__);
	g_video_run_ = 0;
	int ret = 0;

	// USB Camera cleanup (join auto-reconnect thread + safe deinit)
	if (usb_camera_thread_) {
		pthread_join(usb_camera_thread_, NULL);
		usb_camera_thread_ = 0;
	}
	usb_camera_deinit();  // safe: checks fd >= 0 internally
	if (enable_npu || enable_ivs)
		ret |= rkipc_pipe_2_deinit();
	// rk_region_clip_set_callback_register(NULL);
	rk_roi_set_callback_register(NULL);
	if (enable_osd)
		ret |= rkipc_osd_deinit();
	// if (g_enable_vo)
	// 	ret |= rkipc_pipe_vi_vo_deinit();
	if (enable_venc_0) {
		pthread_join(venc_thread_0, NULL);
		ret |= rkipc_pipe_0_deinit();
	}
	if (enable_venc_1) {
		pthread_join(venc_thread_1, NULL);
		ret |= rkipc_pipe_1_deinit();
	}
	if (enable_jpeg) {
		if (rk_param_get_int("video.jpeg:enable_cycle_snapshot", 0)) {
			cycle_snapshot_flag = 0;
			pthread_join(cycle_snapshot_thread_id, NULL);
		}
		pthread_join(jpeg_venc_thread_id, NULL);
		ret |= rkipc_pipe_jpeg_deinit();
	}
	ret |= rkipc_vi_dev_deinit();
	if (enable_tcp_video)
		tcp_video_server_deinit();
	tcp_control_server_deinit();
	if (enable_rtmp)
		ret |= rkipc_rtmp_deinit();
	if (enable_rtsp)
		ret |= rkipc_rtsp_deinit();

	return ret;
}

extern char *rkipc_iq_file_path_;
int rk_video_restart() {
	int ret;
	ret = rk_storage_deinit();
	ret |= rk_video_deinit();
	if (rk_param_get_int("video.source:enable_aiq", 1))
		ret |= rk_isp_deinit(0);
	if (rk_param_get_int("video.source:enable_aiq", 1)) {
		ret |= rk_isp_init(0, rkipc_iq_file_path_);
		if (rk_param_get_int("isp:init_form_ini", 1))
			ret |= rk_isp_set_from_ini(0);
	}
	ret |= rk_video_init();
	ret |= rk_storage_init();

	return ret;
}
