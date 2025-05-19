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

void MainInit(void);

void MainTask(void);

void UpdateData();
void Run();
void RunOnControl();
void RunOnAuto();
void SetSearchRef();
void RunOnGrab();
void RunOnReturn();
void RunOnDead();
void ResetPID();
void DefinePath();
float Bound(float x, float lim1, float lim2);
float LimDiff(float ref_vel, float curr_vel, float max_diff);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __MAIN_TASK_HPP__ */