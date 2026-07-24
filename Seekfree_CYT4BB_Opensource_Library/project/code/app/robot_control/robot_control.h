#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include "../../common/types.h"
#include "../../control/leg/leg_cmd_solve.h"
#include "../../control/leg/leg_pid_control.h"
#include "../../control/leg/vmc_calculate.h"
#include "../../control/balance/pitch_balance.h"
#include "../../lib/pid/pid_calculate.h"
#include "track_elements.h"

extern Motor_cmd_duty_t g_motor_cmd;

extern Sensor_data_t g_sensor_data;

extern Move_cmd_t g_move_cmd;

/* Ctrl_Input_t 供 ISR 中访问 */
extern Ctrl_Input_t g_ctrl;

/* 俯仰平衡 PID (jump.c 需要共享) */
extern PID_Controller_t g_speed_pid;
extern PID_Controller_t g_pitch_angle_pid;
extern PID_Controller_t g_pitch_gyro_pid;

/* 偏航角速度 PID (jump.c 需要共享) */
extern PID_Controller_t g_yaw_angle_pid;
extern PID_Controller_t g_yaw_pid;

/* 腿关节 PID (jump.c LAUNCH 阶段临时拉高增益) */
extern Leg_PID_t g_leg_left_pid;
extern Leg_PID_t g_leg_right_pid;

/* 腿速度/横滚 PID (track_elements 颠簸路段需要切增益) */
extern PID_Controller_t g_leg_speed_pid;
extern PID_Controller_t g_leg_roll_pid;

void robot_control_init(void);

void sensor_update(const Sensor_data_t *sensor);

void command_update(const Move_cmd_t *cmd);

void control_task(void);

void sensor_cmd_update(const Ctrl_Input_t *ctrl, Sensor_data_t *sensor, Move_cmd_t *cmd);

void robot_control_reset_balance_pid(void);

/* 复位腿速度环 PID 积分 (落地时清除空中积累的轮速误差) */
void robot_control_reset_leg_speed_pid(void);

/* 复位腿位置 PID (正常控制恢复时清除跳跃阶段残留的积分) */
void robot_control_reset_leg_pid(void);

/* 公共辅助: 标称位形 + 雅可比求解 → 关节目标 (供 jump.c 复用) */
void leg_offset_to_joint_target(LegSide_t side,
    const Foot_position_t *foot_pos, Leg_Target_t *target);

/* 腿速度环 + 横滚补偿 (供 jump.c 地面相位复用) */
void robot_control_leg_speed_feedback(const Sensor_data_t *sensor,
    Foot_position_t *left, Foot_position_t *right);

/* 里程计: 弧线积分位姿 (x,y,θ) + 路径长度 */
float robot_control_get_x(void);
float robot_control_get_y(void);
float robot_control_get_theta(void);
float robot_control_get_distance(void);
float robot_control_get_yaw(void);
float robot_control_get_speed_mps(void);

/* 跳跃+下坡元素完成信号: 下坡保护结束时置 true, 导航读取后自行清零 */
extern bool g_jump_element_done;

#endif /* ROBOT_CONTROL_H */
