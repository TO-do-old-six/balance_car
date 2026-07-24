#ifndef TRACK_ELEMENTS_H
#define TRACK_ELEMENTS_H

#include "types.h"
#include "../../control/leg/leg_pid_control.h"
#include <stdbool.h>

/* ===== 1. 原地旋转720度 ===== */
void  track_rotate720_init(void);
void  track_rotate720_start(void);
void  track_rotate720_start_with_target(float target_direction);
bool  track_rotate720_is_active(void);
bool  track_rotate720_is_done(void);
bool  track_rotate720_should_suppress_odom(void);
void  track_rotate720_reset(void);
void  track_rotate720_update(Sensor_data_t *sensor, Move_cmd_t *cmd);

/* ===== 2. 单边桥与爬坡 (固定归一化速度) ===== */
void  track_bridge_climb_init(void);
/* 爬坡: 固定速度 -0.4, 无 roll 闭环 */
void  track_bridge_climb_activate(void);
void  track_bridge_climb_deactivate(void);
bool  track_bridge_climb_is_active(void);
void  track_bridge_climb_apply(Move_cmd_t *cmd);
/* 单边桥: 速度 -0.3, roll PID (KP=-20) 保持车身水平 */
void  track_single_bridge_activate(void);
void  track_single_bridge_deactivate(void);

/* ===== 3. 颠簸路段 (抬底盘越障) =====
 *   策略: 双腿伸腿(负Y)抬高底盘 + 平衡PID等比缩小 + 低速通过
 *   适用于密集路肩: 2cm高 / 2.5cm宽, 间距 ~10cm, 总长 >=1m */

void  track_bumpy_init(void);
void  track_bumpy_activate(void);
void  track_bumpy_deactivate(void);
bool  track_bumpy_is_active(void);
float track_bumpy_get_speed(void);

/* 增益切换 */
void  track_bumpy_apply_compliance(void);
void  track_bumpy_restore_stiffness(void);

/* 平衡PID切换 (腿伸长后等效摆长变大, 需降低平衡增益防振荡) */
void  track_bumpy_apply_balance_pid(void);
void  track_bumpy_restore_balance_pid(void);

/* 状态机: 每周期调用 */
void  track_bumpy_update(const Sensor_data_t *sensor);

/* 当前偏航偏置 (rad), 叠加到 target_direction */
float track_bumpy_get_yaw_bias(void);

/* 当前左/右腿收腿量 (mm), 叠加到足端 Y (负 = 伸腿 = 提底盘) */
float track_bumpy_get_left_lift(void);
float track_bumpy_get_right_lift(void);

#endif /* TRACK_ELEMENTS_H */
