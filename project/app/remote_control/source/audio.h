/*
 * audio.h - 双向对讲音频模块接口
 *
 * 使用方式：
 *   audio_init()    - 初始化音频和 TCP 服务器
 *   audio_start()   - 启动音频服务线程
 *   audio_stop()    - 停止音频服务
 *   audio_cleanup() - 清理资源
 */

#ifndef __AUDIO_H__
#define __AUDIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化音频模块
 * 
 * @return 0 成功，-1 失败
 */
int audio_init(void);

/**
 * 启动音频服务线程
 * 
 * @return 0 成功，-1 失败
 */
int audio_start(void);

/**
 * 停止音频服务
 */
void audio_stop(void);

/**
 * 清理音频资源
 */
void audio_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_H__ */
