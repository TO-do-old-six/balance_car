#include "robot_control.h"
#include "track_elements.h"
#include "small_driver_uart_control.h"
#include "jump.h"
#include "../../lib/pid/pid_calculate.h"
#include "../../common/types.h"
#include "../remote/remote_debug.h"
#include "../navigation/nav_engine.h"
#include "../../control/leg/angle_offset.h"
#include "../../control/leg/leg_pid_control.h"
#include "../../control/leg/leg_vmc_control.h"
#include "../../control/leg/kinematics.h"
#include "../../control/leg/jacobian.h"
#include "../../control/balance/pitch_balance.h"
#include <math.h>

static Foot_position_t foot_position_left;

static Foot_position_t foot_position_right;

/* ─── 里程计: 弧线积分 (x, y, θ) ─── */
static float g_odom_x     = 0.0f;  /* 位置 x (m) */
static float g_odom_y     = 0.0f;  /* 位置 y (m) */
static float g_odom_theta = 0.0f;  /* 朝向 (rad) */
static float g_odom_path  = 0.0f;  /* 总路径长度 (m), 用于距离判断 */
static float g_odom_scale = 1.0f;  /* 标定系数, 遥控可调 */

/* ─── 跳跃后下坡保护: 冷却期结束后前 1.5m 禁止刹车 ─── */
static bool  g_downhill_protect_active = false;
static float g_downhill_start_path     = 0.0f;
static bool  g_was_in_cooldown         = false;

/* 跳跃+下坡元素完成信号: 下坡保护结束时置 true */
bool g_jump_element_done = false;

float robot_control_get_x(void)        { return g_odom_x; }
float robot_control_get_y(void)        { return g_odom_y; }
float robot_control_get_theta(void)    { return g_odom_theta; }
float robot_control_get_distance(void) { return g_odom_path; }
float robot_control_get_yaw(void)      { return g_odom_theta; }
float robot_control_get_speed_mps(void)
{
    float wheel_radius_m = LEG_WHEEL_RADIUS * 0.001f;
    return (g_sensor_data.motor_left_speed +
            g_sensor_data.motor_right_speed) * 0.5f * wheel_radius_m;
}

/* ─── 控制模式选择 ───
 *  USE_VMC = 0 : 当前 PID 方案 (默认)
 *  USE_VMC = 1 : 修复后的 VMC 方案 (需现场调参)
 */
#define USE_VMC 0
#define REMOTE_STEER_GAIN_RAD 6.00f
#define CMD_VELOCITY_ACCEL_RATE 8.0f
#define CMD_VELOCITY_DECEL_RATE 14.0f
#define CMD_VELOCITY_BRAKE_RATE 30.0f
#define CMD_VELOCITY_STOP_EPS   0.002f
#define AIR_BALANCE_GAIN      1.0f    /* 空中平衡环缩放, 轮子反作用力矩稳定身体 */
/* 腿部关节 PID 控制器 */
Leg_PID_t g_leg_left_pid, g_leg_right_pid;

/* 腿部关节目标角度 (相对限位的偏移量, rad) */
static Leg_Target_t g_leg_target_left, g_leg_target_right;

#if USE_VMC
static VMC_Config_t g_vmc_config;
#endif

PID_Controller_t g_pitch_angle_pid, g_pitch_gyro_pid, g_speed_pid;
PID_Controller_t g_yaw_angle_pid, g_yaw_pid;
PID_Controller_t g_leg_speed_pid, g_leg_roll_pid;
static float g_brake_p_gain = -0.3f;  /* 刹车 P 增益, slot 8 可调 */

void robot_control_init(void){
    //pitch的PID
    pid_init(&g_pitch_angle_pid, 25.0f, 0.0f, 0.0f, ROBOT_CONTROL_DT, 270.0f, 0.0f);
    pid_init(&g_pitch_gyro_pid,  -1100.0f, 0.0f, 3.00f, ROBOT_CONTROL_DT, 10000.0f, 3000.0f);
    pid_init(&g_speed_pid,       -0.0f, -0.00f, -0.00f, ROBOT_CONTROL_DT, 0.48f, 0.082f);  /* 输出倾角(rad): ±8°限幅, ±3°积分 */
    //偏航串级: 外环角度, 内环角速度
    pid_init(&g_yaw_angle_pid,    5.0f, 0.5f, 0.0f, ROBOT_CONTROL_DT, MAX_YAW_RATE, 1.0f);
    pid_init(&g_yaw_pid,       2150.0f, 10.0f, 0.0f, ROBOT_CONTROL_DT, 10000.0f, 2000.0f);
    //针对腿位置的PID，需要在完整的车上调试
    pid_init(&g_leg_speed_pid, 552.0f, 0.97f, 3.81f, ROBOT_CONTROL_DT, 75.0f, 50.0f);
    pid_init(&g_leg_roll_pid,  0.0f, 0.0f, 0.0f, ROBOT_CONTROL_DT, 1.0f, 0.5f);

    //关节角度的PID，需要在完整的车上调试
    leg_pid_init(&g_leg_left_pid,  1200.0f, 0.0f, 4.0f, 10000.0f, 0.0f);
    leg_pid_init(&g_leg_right_pid, 1200.0f, 0.0f, 4.0f, 10000.0f, 0.0f);

#if USE_VMC
    g_vmc_config.kp = 0.25f;
    g_vmc_config.kd = 0.0000f;
#endif

    //初始偏移，根据实际情况改一下重心
    g_leg_target_left.front  = 0.4f;
    g_leg_target_left.back   = -0.4f;
    g_leg_target_right.front = 0.4f;
    g_leg_target_right.back  = -0.4f;

    remote_debug_bind(0, &g_pitch_angle_pid.kp);
    remote_debug_bind(1, &g_pitch_gyro_pid.kp);
    remote_debug_bind(2, &g_pitch_gyro_pid.kd);
    remote_debug_bind(3, &g_speed_pid.kp);
    remote_debug_bind(4, &g_leg_left_pid.front.kp);
    remote_debug_bind(5, &g_leg_roll_pid.kp);
    remote_debug_bind(6, &g_yaw_pid.kp);
    remote_debug_bind(7, &g_yaw_pid.kd);
    remote_debug_bind(8, &g_brake_p_gain);

#if USE_VMC
    remote_debug_bind(6, &g_vmc_config.kp);
    remote_debug_bind(7, &g_vmc_config.kd);
#endif

    jump_init();
    track_rotate720_init();
    track_bridge_climb_init();
    track_bumpy_init();

}

/*---------------------------------------------------------------------------*/
void sensor_update(const Sensor_data_t *sensor){
    g_sensor_data = *sensor;
}

/*---------------------------------------------------------------------------*/
void command_update(const Move_cmd_t *cmd){
    g_move_cmd = *cmd;
}

/*---------------------------------------------------------------------------*/
void robot_control_reset_balance_pid(void){
    pid_reset(&g_pitch_angle_pid);
    pid_reset(&g_pitch_gyro_pid);
    pid_reset(&g_speed_pid);
}

void robot_control_reset_leg_speed_pid(void){
    pid_reset(&g_leg_speed_pid);
    pid_reset(&g_leg_roll_pid);
}

void robot_control_reset_leg_pid(void){
    pid_reset(&g_leg_left_pid.front);
    pid_reset(&g_leg_left_pid.back);
    pid_reset(&g_leg_right_pid.front);
    pid_reset(&g_leg_right_pid.back);
}

/*---------------------------------------------------------------------------*/
/** 腿位置速度环反馈: 根据轮速调整足端 X, 根据横滚调整足端 Y */
void robot_control_leg_speed_feedback(const Sensor_data_t *sensor,
    Foot_position_t *left, Foot_position_t *right)
{
    Move_cmd_t dummy = { .target_direction = 0.0f, .target_distance = 0.0f, .target_speed = 0.0f, .target_roll = 0.0f, .target_height = 0.0f };
    Foot_position_t fb_left, fb_right;
    leg_cmd_solve(&dummy, sensor, &g_leg_speed_pid, &g_leg_roll_pid,
                  &fb_left, &fb_right);
    left->x  += fb_left.x;
    left->y  += fb_left.y;
    right->x += fb_right.x;
    right->y += fb_right.y;
}

/*---------------------------------------------------------------------------*/
/** 将足端偏移通过雅可比逆解转换为关节目标角度 */
void leg_offset_to_joint_target(LegSide_t side,
    const Foot_position_t *foot_pos, Leg_Target_t *target)
{
    float nom_front, nom_back, sign_front, sign_back;

    if (side == LEG_LEFT) {
        nom_front  = LEG_NOM_FRONT_L;
        nom_back   = LEG_NOM_BACK_L;
        sign_front = 1.0f;
        sign_back  = 1.0f;
    } else {
        nom_front  = LEG_NOM_FRONT_R;
        nom_back   = LEG_NOM_BACK_R;
        sign_front = RIGHT_ABS_FRONT_SIGN;
        sign_back  = RIGHT_ABS_BACK_SIGN;
    }

    Joint_angle_t abs_nominal;
    float J[2][2], dth1, dth2;

    abs_nominal.left_motor_angle  = M_PI_2 + sign_front * nom_front;
    abs_nominal.right_motor_angle = M_PI_2 + sign_back  * nom_back;

    if (five_bar_jacobian(&abs_nominal, J) == 0 &&
        five_bar_jacobian_solve(J, &dth1, &dth2,
            foot_pos->x, foot_pos->y) == 0) {
        target->front = nom_front + sign_front * dth1;
        target->back  = nom_back  + sign_back  * dth2;
    } else {
        /* 奇异 → 保持标称 */
        target->front = nom_front;
        target->back  = nom_back;
    }

    /* 关节限位: 不发出超过180°的目标 */
    target->front = CLAMP(target->front, -SAFE_JOINT_MAX_RAD, SAFE_JOINT_MAX_RAD);
    target->back  = CLAMP(target->back,  -SAFE_JOINT_MAX_RAD, SAFE_JOINT_MAX_RAD);
}

/*---------------------------------------------------------------------------*/
/** 安全检查: 倾角/关节角度超限时切断所有电机输出, 返回 false
 *  故障会锁存, 一旦触发则后续所有周期永久停机, 需复位重启.
 */
static bool g_safety_fault = false;

static bool safety_check(const Sensor_data_t *sensor, Motor_cmd_duty_t *motor_cmd)
{
    /* ── 倾角保护 ── */
    float pitch_deg = sensor->angle_pitch * RAD_TO_DEG;
    float roll_deg  = sensor->angle_roll  * RAD_TO_DEG;

    if (g_safety_fault) {
        goto fault;
    }

    /* ── 倾角保护 (颠簸路段期间关闭, 防止路肩导致的姿态振荡误触发) ── */
    if (!track_bumpy_is_active()) {
        if (fabsf(pitch_deg) > SAFE_PITCH_MAX_DEG ||
            fabsf(roll_deg)  > SAFE_ROLL_MAX_DEG) {
            g_safety_fault = true;
            goto fault;
        }
    }

    /* ── 关节角度保护 (相对限位的转角) ── */
    if (fabsf(sensor->joint_left_front_angle)  > SAFE_JOINT_MAX_RAD ||
        fabsf(sensor->joint_left_back_angle)   > SAFE_JOINT_MAX_RAD ||
        fabsf(sensor->joint_right_front_angle) > SAFE_JOINT_MAX_RAD ||
        fabsf(sensor->joint_right_back_angle)  > SAFE_JOINT_MAX_RAD) {
        g_safety_fault = true;
        goto fault;
    }

    return true;

fault:
    motor_cmd->left_motor_pwm       = 0;
    motor_cmd->right_motor_pwm      = 0;
    motor_cmd->left_front_joint_pwm  = 0;
    motor_cmd->left_back_joint_pwm   = 0;
    motor_cmd->right_front_joint_pwm = 0;
    motor_cmd->right_back_joint_pwm  = 0;
    return false;
}

/*---------------------------------------------------------------------------*/
void control_task(void){
    Sensor_data_t sensor_local = g_sensor_data;
    Move_cmd_t     cmd_local   = g_move_cmd;

    if (!safety_check(&sensor_local, &g_motor_cmd)) {
        return;
    }

    bool airborne = jump_is_airborne();

    /* ── 速度指令覆盖: balance_control 和 leg_cmd_solve 共用 ── */
    if (!airborne) {
        track_bridge_climb_apply(&cmd_local);
        if (track_bumpy_is_active()) {
            cmd_local.target_speed = track_bumpy_get_speed();
        }
    }

    /* ── 轮式平衡 + 偏航 ── */
    if (!airborne) {
        {
            float pitch_target = 0.0f;

            /* ── 跳跃后下坡过渡检测: 记录冷却期开始→结束, 每周期运行 ── */
            if (!g_was_in_cooldown && jump_is_in_cooldown()) {
                g_was_in_cooldown = true;
            }
            if (g_was_in_cooldown && !jump_is_in_cooldown()) {
                g_downhill_protect_active = true;
                g_downhill_start_path = g_odom_path;
                g_was_in_cooldown = false;
            }

            if (!jump_is_active() && !jump_is_in_cooldown() && !track_bridge_climb_is_active()) {
                /* speed_control 已关闭 (g_speed_pid 增益均为 0), 用纯 P 刹车 */

                /* ── 下坡里程保护: 冷却期结束后前 1.5m 禁止刹车 ── */
                bool downhill_protected = false;
                if (g_downhill_protect_active) {
                    if ((g_odom_path - g_downhill_start_path) < 1.5f) {
                        downhill_protected = true;
                    } else {
                        g_downhill_protect_active = false;
                        g_jump_element_done = true;  /* 通知导航: 跳跃+下坡元素完成 */
                    }
                }

                if (!downhill_protected) {
                    float speed_norm = (sensor_local.motor_left_speed + sensor_local.motor_right_speed) / 120.0f;

                    bool stop_request    = (fabsf(cmd_local.target_speed) < 0.02f && fabsf(speed_norm) > 0.015f);
                    bool reverse_request = (cmd_local.target_speed * speed_norm < -0.0003f);

                    if (stop_request || reverse_request) {
                        /* 跳跃全过程关闭刹车 P 增益 */
                        float brake_gain = (jump_is_active() || jump_is_in_cooldown()) ? 0.0f : g_brake_p_gain;
                        pitch_target = -brake_gain * speed_norm;
                        pitch_target = CLAMP(pitch_target, -0.25f, 0.25f);
                    }
                }
            }
            float pwm_base = balance_control(&sensor_local, &g_pitch_angle_pid, &g_pitch_gyro_pid, pitch_target);
            g_motor_cmd.left_motor_pwm  = ROUND(pwm_base);
            g_motor_cmd.right_motor_pwm = ROUND(-pwm_base);
        }

        /* ── 720° 原地旋转: 设置 target_direction + target_roll ── */
        track_rotate720_update(&sensor_local, &cmd_local);

        /* ── 颠簸路段: 伸腿抬底盘 + 柔顺KP + 低速 ── */
        {
            static bool bumpy_was_active = false;
            if (track_bumpy_is_active()) {
                track_bumpy_apply_compliance();   /* 腿PID */
                track_bumpy_apply_balance_pid();  /* 平衡PID: 腿伸长后等效摆长变大 */
                track_bumpy_update(&sensor_local);
                cmd_local.target_direction += track_bumpy_get_yaw_bias();
                bumpy_was_active = true;
            } else if (bumpy_was_active) {
                track_bumpy_restore_stiffness();
                track_bumpy_restore_balance_pid();
                bumpy_was_active = false;
            }
        }

        /* 旋转完成时复位 yaw PID, 锁定当前朝向 */
        {
            static bool was_rotate720_done = false;
            bool is_done = track_rotate720_is_done();
            if (is_done && !was_rotate720_done) {
                pid_reset(&g_yaw_angle_pid);
                pid_reset(&g_yaw_pid);
            }
            was_rotate720_done = is_done;
        }

        /* 差速转向: 串级控制 (外环角度, 内环角速度), 直接使用 target_direction */
        {
            float yaw_cur   = sensor_local.angle_yaw;
            float yaw_error = cmd_local.target_direction - yaw_cur;
            while (yaw_error >  M_PI) yaw_error -= 2.0f * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;
            yaw_cur = cmd_local.target_direction - yaw_error;

            /* 方向大幅跳变时复位 yaw PID (平滑积分不触发) */
            static float prev_target_direction = 0.0f;
            if (fabsf(cmd_local.target_direction - prev_target_direction) > 0.5f) {
                pid_reset(&g_yaw_angle_pid);
                pid_reset(&g_yaw_pid);
            }
            prev_target_direction = cmd_local.target_direction;

            /* 外环: 角度误差 → 角速度修正 */
            float rate_correction = pid_calculate(&g_yaw_angle_pid, cmd_local.target_direction, yaw_cur);

            /* 内环: 角速度修正 → 差模 PWM */
            float yaw_out = pid_calculate(&g_yaw_pid, rate_correction, -sensor_local.gyro_yaw);
            int   yaw_pwm = ROUND(yaw_out);
            g_motor_cmd.left_motor_pwm  -= yaw_pwm;
            g_motor_cmd.right_motor_pwm -= yaw_pwm;
        }

        /* 自动压弯: 根据 yaw rate 计算 body roll, 转弯时内侧下沉 */
        /* 旋转时由 track_rotate720_update 设置增强系数, 此处跳过 */
        if (!track_rotate720_is_active()) {
            float roll_from_turn = -0.2f * sensor_local.gyro_yaw / MAX_YAW_RATE;
            cmd_local.target_roll = CLAMP(roll_from_turn, -1.0f, 1.0f);
        }
    } else {
        /* 空中: 关闭速度环, 只保直立, 轮子反作用力矩稳定身体 */
        {
            float pwm_base = balance_control(&sensor_local, &g_pitch_angle_pid, &g_pitch_gyro_pid, 0.0f);
            g_motor_cmd.left_motor_pwm  = ROUND(pwm_base);
            g_motor_cmd.right_motor_pwm = ROUND(-pwm_base);
        }
        g_motor_cmd.left_motor_pwm  = (int)(g_motor_cmd.left_motor_pwm  * AIR_BALANCE_GAIN);
        g_motor_cmd.right_motor_pwm = (int)(g_motor_cmd.right_motor_pwm * AIR_BALANCE_GAIN);
    }

    /* ── 足端位置计算 ── */
    if (!airborne) {
        /* 跳跃地面阶段: 用接近速度驱动 g_leg_speed_pid */
        if (jump_is_active()) {
            cmd_local.target_speed = jump_get_approach_speed();
        }
        /* 冷却期: 低速仅由 g_leg_speed_pid 驱动 */
        if (jump_is_in_cooldown()) {
            cmd_local.target_speed = -0.0f;
        }
        /* 颠簸路段: 低速 (由 track_bumpy_get_speed 统一控制) */
        if (track_bumpy_is_active()) {
            cmd_local.target_speed = track_bumpy_get_speed();
        }
        /* 起跳前自稳+下蹲: 保持前向接近速度, 蹬地和着地阶段不干预 */
        if (jump_is_stabilizing() || jump_is_squatting()) {
            float app_speed = jump_get_approach_speed();
            if (app_speed != 0.0f) {
                cmd_local.target_speed = app_speed;
            }
        }
        /* 跳跃后下坡保护: 冷却期结束后前 1.5m, 限速行驶, 不用刹车 */
        if (g_downhill_protect_active) {
            cmd_local.target_speed = 0.05f;
        }
        leg_cmd_solve(&cmd_local, &sensor_local, &g_leg_speed_pid, &g_leg_roll_pid,
            &foot_position_left, &foot_position_right);

        /* 跳跃期间关闭 roll 闭环, 由跳跃状态机自己控制 Y */
        if (jump_is_active()) {
            foot_position_left.y  = 0.0f;
            foot_position_right.y = 0.0f;
        }
    } else {
        foot_position_left.x  = 0.0f;
        foot_position_left.y  = 0.0f;
        foot_position_right.x = 0.0f;
        foot_position_right.y = 0.0f;
    }

    /* ── 跳跃腿轨迹叠加 (在 leg_cmd_solve 之后修改足端 Y + X 偏置) ── */
    if (jump_is_active()) {
        jump_leg_overlay(&foot_position_left, &foot_position_right, &sensor_local);
    }

    /* ── 颠簸路段: 单侧收腿, 交替爬升 ── */
    if (track_bumpy_is_active()) {
        foot_position_left.y  += track_bumpy_get_left_lift();
        foot_position_right.y += track_bumpy_get_right_lift();
    }

    /* ── 腿控制 ── */
#if USE_VMC
    /* ── VMC 方案（修复了角度偏置/符号/目标计算） ── */
    leg_vmc_control(&g_vmc_config, &sensor_local,
                    &foot_position_left, &foot_position_right,
                    &g_motor_cmd);
#else

    leg_offset_to_joint_target(LEG_LEFT,  &foot_position_left,  &g_leg_target_left);
    leg_offset_to_joint_target(LEG_RIGHT, &foot_position_right, &g_leg_target_right);

    leg_pid_control(&g_leg_left_pid, &g_leg_target_left,
                    &sensor_local, LEG_LEFT, &g_motor_cmd);

    leg_pid_control(&g_leg_right_pid, &g_leg_target_right,
                    &sensor_local, LEG_RIGHT, &g_motor_cmd);
#endif

    /* PWM 输出低通滤波, 抑制高频抖动 */
    {
        static float lm_filt = 0.0f, rm_filt = 0.0f;
        static float lf_filt = 0.0f, lb_filt = 0.0f;
        static float rf_filt = 0.0f, rb_filt = 0.0f;

        lm_filt += 0.2f * (g_motor_cmd.left_motor_pwm       - lm_filt);
        rm_filt += 0.2f * (g_motor_cmd.right_motor_pwm      - rm_filt);
        lf_filt += 0.2f * (g_motor_cmd.left_front_joint_pwm  - lf_filt);
        lb_filt += 0.2f * (g_motor_cmd.left_back_joint_pwm   - lb_filt);
        rf_filt += 0.2f * (g_motor_cmd.right_front_joint_pwm - rf_filt);
        rb_filt += 0.2f * (g_motor_cmd.right_back_joint_pwm  - rb_filt);

        g_motor_cmd.left_motor_pwm       = (int)lm_filt;
        g_motor_cmd.right_motor_pwm      = (int)rm_filt;
        g_motor_cmd.left_front_joint_pwm  = (int)lf_filt;
        g_motor_cmd.left_back_joint_pwm   = (int)lb_filt;
        g_motor_cmd.right_front_joint_pwm = (int)rf_filt;
        g_motor_cmd.right_back_joint_pwm  = (int)rb_filt;
    }
}

/*---------------------------------------------------------------------------*/
/* 折返测试: 平衡 → 前进 → 180°掉头 → 循环, 纯距离控制 */
#define ZF_FWD_DIST_M      1.0f    /* 单程前进距离 (m) */
#define ZF_FWD_SPEED       0.3f    /* 巡航速度 */
#define ZF_START_DELAY     5.0f    /* 启动等待 (s) */
#define ZF_SEG_COUNT       4       /* 总段数 (2个来回) */

static void backforth_test(Move_cmd_t *cmd, const Sensor_data_t *sensor, float elapsed) {
    static int  segment     = 0;   /* 当前段 0~(ZF_SEG_COUNT-1) */
    static bool is_turning  = false;
    static float start_path = 0.0f;
    static float turn_sum   = 0.0f;
    static float prev_yaw   = 0.0f;

    /* 启动延迟: 原地平衡 */
    if (elapsed < ZF_START_DELAY) {
        cmd->target_direction = 0.0f;
        cmd->target_speed     = 0.0f;
        cmd->target_distance  = 0.0f;
        start_path = g_odom_path;  /* 同步里程起点 */
        return;
    }

    /* 全部段完成 */
    if (segment >= ZF_SEG_COUNT) {
        float dir = (segment & 1) ? (float)M_PI : 0.0f;
        cmd->target_direction = dir;
        cmd->target_speed     = 0.0f;
        cmd->target_distance  = 0.0f;
        return;
    }

    if (!is_turning) {
        /* ── 前进段 ── */
        float dir = (segment & 1) ? (float)M_PI : 0.0f;  /* 段0→0, 段1→π, 段2→0, 段3→π */
        cmd->target_direction = dir;
        cmd->target_speed     = ZF_FWD_SPEED;
        cmd->target_distance  = 0.0f;

        if ((g_odom_path - start_path) >= ZF_FWD_DIST_M) {
            is_turning = true;
            turn_sum   = 0.0f;
            prev_yaw   = sensor->angle_yaw;
        }
    } else {
        /* ── 掉头段 ── */
        float target = (segment & 1) ? 0.0f : (float)M_PI;  /* 掉头到对面方向 */
        cmd->target_direction = target;
        cmd->target_speed     = 0.0f;
        cmd->target_distance  = 0.0f;

        float dy = sensor->angle_yaw - prev_yaw;
        while (dy >  M_PI) dy -= 2.0f * (float)M_PI;
        while (dy < -M_PI) dy += 2.0f * (float)M_PI;
        turn_sum += fabsf(dy);
        prev_yaw = sensor->angle_yaw;

        if (turn_sum >= (float)M_PI * 0.95f) {
            is_turning = false;
            segment++;
            start_path = g_odom_path;  /* 下一段从当前位置开始计里程 */
        }
    }
}

/*---------------------------------------------------------------------------*/
/* 画方测试: 前进 → 右转90° → 循环, 纯距离控制 */
#define SQ_FWD_DIST_M      1.0f    /* 单边前进距离 (m) */
#define SQ_FWD_SPEED       0.5f    /* 巡航速度 */
#define SQ_START_DELAY     5.0f    /* 启动等待 (s) */
#define SQ_SEG_COUNT       8       /* 总段数 (2个正方形) */

static void square_test(Move_cmd_t *cmd, const Sensor_data_t *sensor, float elapsed) {
    static int  segment     = 0;
    static bool is_turning  = false;
    static float start_path = 0.0f;
    static float turn_sum   = 0.0f;
    static float prev_yaw   = 0.0f;
    static float cur_dir    = 0.0f;  /* 当前前进方向 */

    if (elapsed < SQ_START_DELAY) {
        cmd->target_direction = 0.0f;
        cmd->target_speed     = 0.0f;
        cmd->target_distance  = 0.0f;
        start_path = g_odom_path;
        cur_dir    = 0.0f;
        return;
    }

    if (segment >= SQ_SEG_COUNT) {
        cmd->target_direction = cur_dir;
        cmd->target_speed     = 0.0f;
        cmd->target_distance  = 0.0f;
        return;
    }

    if (!is_turning) {
        /* ── 前进段 ── */
        cmd->target_direction = cur_dir;
        cmd->target_speed     = SQ_FWD_SPEED;
        cmd->target_distance  = 0.0f;

        if ((g_odom_path - start_path) >= SQ_FWD_DIST_M) {
            is_turning = true;
            turn_sum   = 0.0f;
            prev_yaw   = sensor->angle_yaw;
            cur_dir   += (float)M_PI_2;  /* 右转90° */
            while (cur_dir >  M_PI) cur_dir -= 2.0f * (float)M_PI;
            while (cur_dir < -M_PI) cur_dir += 2.0f * (float)M_PI;
        }
    } else {
        /* ── 右转90° ── */
        cmd->target_direction = cur_dir;
        cmd->target_speed     = 0.0f;
        cmd->target_distance  = 0.0f;

        float dy = sensor->angle_yaw - prev_yaw;
        while (dy >  M_PI) dy -= 2.0f * (float)M_PI;
        while (dy < -M_PI) dy += 2.0f * (float)M_PI;
        turn_sum += fabsf(dy);
        prev_yaw = sensor->angle_yaw;

        if (turn_sum >= (float)M_PI_2 * 0.95f) {
            is_turning = false;
            segment++;
            start_path = g_odom_path;
        }
    }
}

/*---------------------------------------------------------------------------*/
#define ENABLE_SQUARE_TEST 0

void sensor_cmd_update(const Ctrl_Input_t *ctrl, Sensor_data_t *sensor, Move_cmd_t *cmd){

    /* --- 传感器数据桥接 --- */
    sensor->angle_pitch    = ctrl->body_pitch;
    sensor->angle_roll     = ctrl->body_roll - ROLL_ANGLE_OFFSET_DEG * DEG_TO_RAD;
    sensor->angle_yaw      = ctrl->body_yaw;
    sensor->gyro_pitch     = ctrl->gyro_pitch_rate;
    sensor->gyro_yaw       = ctrl->gyro_yaw_rate;
    sensor->gyro_roll      = ctrl->gyro_roll_rate;
    sensor->accel_z        = ctrl->accel_z;

    /* 上电第一次读到的 yaw 作为 0 点 */
    {
        static float yaw_zero = 0.0f;
        static bool  yaw_zero_set = false;
        if (!yaw_zero_set) {
            yaw_zero = sensor->angle_yaw;
            yaw_zero_set = true;
        }
        sensor->angle_yaw -= yaw_zero;
        while (sensor->angle_yaw >  M_PI) sensor->angle_yaw -= 2.0f * M_PI;
        while (sensor->angle_yaw < -M_PI) sensor->angle_yaw += 2.0f * M_PI;
    }

    /* ── 里程计: 角度差分弧线积分 (x, y, θ) ── */
    {
        /* 角度读取 + 精度补偿: 驱动板发送 0.1° 分辨率,
         * 但 ISR 回调统一按 /100.0f 解码, 需 *10.0f 恢复真实角度 */
        float left_deg  = small_driver_value.receive_left_angle_data  * 10.0f;
        float right_deg = small_driver_value.receive_right_angle_data * 10.0f;

        static float prev_l = 0.0f, prev_r = 0.0f;
        static bool  odom_init = false;
        float dl, dr, dist;

        if (!odom_init) {
            prev_l = left_deg; prev_r = right_deg;
            odom_init = true;
        } else {
            dl = left_deg  - prev_l;
            dr = right_deg - prev_r;
            /* 360° wrap 处理 */
            if (dl < -180.0f) dl += 360.0f;
            if (dl >  180.0f) dl -= 360.0f;
            if (dr < -180.0f) dr += 360.0f;
            if (dr >  180.0f) dr -= 360.0f;
            prev_l = left_deg; prev_r = right_deg;

            /* 轮速由角度差分计算 (rad/s), 不查询0x02避免切换驱动板通信模式 */
            {
                static float motor_left_filt  = 0.0f;
                static float motor_right_filt = 0.0f;
                float left_raw  = dl * DEG_TO_RAD / ROBOT_CONTROL_DT;
                float right_raw = dr * DEG_TO_RAD / ROBOT_CONTROL_DT * RIGHT_MOTOR_DIR;
                motor_left_filt  += 0.2f * (left_raw  - motor_left_filt);
                motor_right_filt += 0.2f * (right_raw - motor_right_filt);
                sensor->motor_left_speed  = motor_left_filt;
                sensor->motor_right_speed = motor_right_filt;
            }

            if (track_rotate720_should_suppress_odom()) {
                g_odom_theta = sensor->angle_yaw;
            } else {
                /* 平均角增量 → 线距离 (m), 右轮镜像需乘 RIGHT_MOTOR_DIR */
                dist = ((dl + dr * RIGHT_MOTOR_DIR) / 2.0f) * DEG_TO_RAD * LEG_WHEEL_RADIUS * 0.001f;

                /* 路径用绝对值积分 */
                g_odom_path += fabsf(dist) * g_odom_scale;

                /* x, y 弧线积分 */
                float theta_cur  = sensor->angle_yaw;
                float theta_prev = g_odom_theta;
                float d_theta    = theta_cur - theta_prev;
                while (d_theta >  M_PI) d_theta -= 2.0f * M_PI;
                while (d_theta < -M_PI) d_theta += 2.0f * M_PI;

                if (fabsf(d_theta) < 0.002f) {
                    /* 近似直线 */
                    g_odom_x += dist * cosf(theta_prev) * g_odom_scale;
                    g_odom_y += dist * sinf(theta_prev) * g_odom_scale;
                } else {
                    /* 弧线: R = dist/dθ 精确积分 */
                    float R = dist / d_theta;
                    g_odom_x += R * (sinf(theta_cur) - sinf(theta_prev)) * g_odom_scale;
                    g_odom_y += R * (cosf(theta_prev) - cosf(theta_cur)) * g_odom_scale;
                }

                g_odom_theta = theta_cur;
            }
        }
    }

    sensor->joint_left_front_angle  = (float)small_driver_value_leg_left.receive_left_location_data * DEG_TO_RAD;
    sensor->joint_left_back_angle   = (float)small_driver_value_leg_left.receive_right_location_data * DEG_TO_RAD;
    sensor->joint_right_front_angle = (float)small_driver_value_leg_right.receive_left_location_data * DEG_TO_RAD;
    sensor->joint_right_back_angle  = (float)small_driver_value_leg_right.receive_right_location_data * DEG_TO_RAD;

    /* 关节角度低通滤波, 滤除编码器量化噪声, 避免差分速度放大噪声 */
    {
        static float lf_filt = 0.0f, lb_filt = 0.0f, rf_filt = 0.0f, rb_filt = 0.0f;
        lf_filt += 0.5f * (sensor->joint_left_front_angle  - lf_filt);
        lb_filt += 0.5f * (sensor->joint_left_back_angle   - lb_filt);
        rf_filt += 0.5f * (sensor->joint_right_front_angle - rf_filt);
        rb_filt += 0.5f * (sensor->joint_right_back_angle  - rb_filt);
        sensor->joint_left_front_angle  = lf_filt;
        sensor->joint_left_back_angle   = lb_filt;
        sensor->joint_right_front_angle = rf_filt;
        sensor->joint_right_back_angle  = rb_filt;
    }

    //应用标定
    if (angle_offset_is_done()) {
        angle_offset_apply_to_sensor(sensor);
    }

    /* 关节速度由位置差分获得（驱动板持续回传位置数据） */
    {
        static float prev_lf = 0.0f, prev_lb = 0.0f, prev_rf = 0.0f, prev_rb = 0.0f;
        static bool first = true;
        if (first) {
            sensor->joint_left_front_speed  = 0.0f;
            sensor->joint_left_back_speed   = 0.0f;
            sensor->joint_right_front_speed = 0.0f;
            sensor->joint_right_back_speed  = 0.0f;
            first = false;
        } else {
            float dt = ROBOT_CONTROL_DT;
            sensor->joint_left_front_speed  = (sensor->joint_left_front_angle  - prev_lf) / dt;
            sensor->joint_left_back_speed   = (sensor->joint_left_back_angle   - prev_lb) / dt;
            sensor->joint_right_front_speed = (sensor->joint_right_front_angle - prev_rf) / dt;
            sensor->joint_right_back_speed  = (sensor->joint_right_back_angle  - prev_rb) / dt;
        }
        prev_lf = sensor->joint_left_front_angle;
        prev_lb = sensor->joint_left_back_angle;
        prev_rf = sensor->joint_right_front_angle;
        prev_rb = sensor->joint_right_back_angle;
    }

    {
        static float velocity_cmd_smooth = 0.0f;
        float velocity_delta = ctrl->velocity_cmd - velocity_cmd_smooth;
        float velocity_rate = CMD_VELOCITY_ACCEL_RATE;
        float max_delta;

        if (ctrl->velocity_cmd * velocity_cmd_smooth < 0.0f) {
            velocity_rate = CMD_VELOCITY_BRAKE_RATE;
        } else if (fabsf(ctrl->velocity_cmd) < fabsf(velocity_cmd_smooth)) {
            velocity_rate = CMD_VELOCITY_DECEL_RATE;
        }

        max_delta = velocity_rate * ROBOT_CONTROL_DT;

        velocity_delta = CLAMP(velocity_delta, -max_delta, max_delta);
        velocity_cmd_smooth += velocity_delta;
        if (fabsf(ctrl->velocity_cmd) < CMD_VELOCITY_STOP_EPS &&
            fabsf(velocity_cmd_smooth) < CMD_VELOCITY_STOP_EPS) {
            velocity_cmd_smooth = 0.0f;
        }

        cmd->target_speed = velocity_cmd_smooth * TARGET_SPEED_MAX;
    }
    cmd->target_roll      = 0.0f;
    cmd->target_height    = 0.0f;
    cmd->target_distance  = 0.0f;
    /* 持久化 yaw 目标: 上电=当前角度, 摇杆积分修改, 回中锁定 */
    {
        static float persist_yaw = 0.0f;
        static bool  persist_init = false;
        if (!persist_init) {
            persist_yaw = sensor->angle_yaw;
            persist_init = true;
        }
        if (ctrl->yaw_target_valid) {
            persist_yaw = ctrl->yaw_target_rad;
        } else {
            persist_yaw += ctrl->steering_cmd * REMOTE_STEER_GAIN_RAD * ROBOT_CONTROL_DT;
        }
        while (persist_yaw >  M_PI) persist_yaw -= 2.0f * M_PI;
        while (persist_yaw < -M_PI) persist_yaw += 2.0f * M_PI;
        cmd->target_direction = persist_yaw;
    }

#if ENABLE_SQUARE_TEST
    /* 标定完成后才开始计时, 避免标定期消耗折返测试的启动延迟 */
    {
        static float bf_elapsed    = 0.0f;
        static bool  bf_was_done   = false;
        if (angle_offset_is_done()) {
            if (!bf_was_done) {
                bf_elapsed  = 0.0f;
                bf_was_done = true;
            }
            bf_elapsed += ROBOT_CONTROL_DT;
        } else {
            bf_was_done = false;
        }

        /* 画方测试: 使用外部计时 */
        square_test(cmd, sensor, bf_elapsed);
    }
#endif
}
