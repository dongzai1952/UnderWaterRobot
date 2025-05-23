#ifndef __MAIN_TASK_HPP__
#define __MAIN_TASK_HPP__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define PI 3.1415926f
#define kControlPeriod 0.01f //10ms控制周期
#define kDepthSpeed 4.0f  //cm/s
#define kRoundSpeed 20.0f  //rad/s
#define kSpeedMax 30.0f
#define kSpeedX 15.0f
#define kSpeedY 15.0f
#define kPointTH 3.0f  //cm
#define kDepthNormal 15.0f  //cm
#define kDepthGrab 25.0f  //cm
#define kDepthShit 29.0f  //cm
#define kGrabRange 10.0  //cm  抓取搜素范围

void MainInit(void);

void MainTask(void);

void UpdateData();
void Run();
void RunOnControl();
void RunOnAuto();
void SetSearchRef();
void SetGrabRef();
void SetReturnRef();
void RunOnDead();
void RunOnShit();
void ShitInit();
void ResetPID();
void DefinePath();
float Bound(float x, float lim1, float lim2);
float LimDiff(float ref_vel, float curr_vel, float max_diff);
void SwitchAutoState();
void EMOn();  //电磁铁开
void EMOff();  //电磁铁关
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __MAIN_TASK_HPP__ */