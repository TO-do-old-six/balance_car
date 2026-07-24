#include "track_elements.h"
#include "../../lib/pid/pid_calculate.h"
#include "../../common/types.h"
#include "robot_control.h"
#include <math.h>
#include <stddef.h>

/* =========================== 原地旋转720度 =========================== */

#define ROT720_SPEED                 (3.5f * M_PI)  /* 630°/s */
#define ROT720_MIN_SPEED             (0.9f * M_PI)  /* 末端最低目标角速度 */
#define ROT720_SLOWDOWN_ANGLE        (1.0f * M_PI)  /* 剩余半圈开始降速 */
#define ROT720_SLOWDOWN_GAIN         5.0f
#define ROT720_BASE_TURN             (4.0f * M_PI)  /* 先转两整圈 */
#define ROT720_MARGIN                (0.0f * M_PI)  /* 停止角度冗余 */
#define ROT720_LEAN_GAIN             0.35f          /* 压弯系数 (原0.2) */
#define ROT720_SETTLE_LEAN_GAIN      0.20f
#define ROT720_BRAKE_SPEED_TOL_MPS   0.04f
#define ROT720_BRAKE_STABLE_TICKS    60u
#define ROT720_BRAKE_TIMEOUT_TICKS   500u
#define ROT720_SETTLE_YAW_TOL        (2.0f * M_PI / 180.0f)
#define ROT720_SETTLE_RATE_TOL       0.25f
#define ROT720_SETTLE_STABLE_TICKS   80u
#define ROT720_SETTLE_TIMEOUT_TICKS  900u

typedef enum {
    ROTATE720_IDLE = 0,
    ROTATE720_BRAKING,
    ROTATE720_ACTIVE,
    ROTATE720_SETTLING,
    ROTATE720_DONE
} Rotate720State_t;

static Rotate720State_t g_rotate720_state = ROTATE720_IDLE;
static float            g_rotate720_accum = 0.0f;
static float            g_rotate720_target = 0.0f;
static float            g_rotate720_target_accum = ROT720_BASE_TURN;
static float            g_rotate720_dir = 1.0f;
static bool             g_rotate720_target_valid = false;
static float            g_rotate720_final_target = 0.0f;
static bool             g_rotate720_final_target_valid = false;
static float            g_rotate720_lock_target = 0.0f;
static uint16_t         g_rotate720_brake_ticks = 0u;
static uint16_t         g_rotate720_brake_stable_ticks = 0u;
static uint16_t         g_rotate720_settle_ticks = 0u;
static uint16_t         g_rotate720_settle_stable_ticks = 0u;

static float rotate720_wrap_pi(float angle)
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }

    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }

    return angle;
}

void track_rotate720_init(void)
{
    g_rotate720_state  = ROTATE720_IDLE;
    g_rotate720_accum  = 0.0f;
    g_rotate720_target = 0.0f;
    g_rotate720_target_accum = ROT720_BASE_TURN;
    g_rotate720_dir = 1.0f;
    g_rotate720_target_valid = false;
    g_rotate720_final_target = 0.0f;
    g_rotate720_final_target_valid = false;
    g_rotate720_lock_target = 0.0f;
    g_rotate720_brake_ticks = 0u;
    g_rotate720_brake_stable_ticks = 0u;
    g_rotate720_settle_ticks = 0u;
    g_rotate720_settle_stable_ticks = 0u;
}

static void rotate720_start_common(bool final_target_valid,
                                   float final_target)
{
    if (g_rotate720_state == ROTATE720_IDLE) {
        g_rotate720_state  = ROTATE720_BRAKING;
        g_rotate720_accum  = 0.0f;
        g_rotate720_target = 0.0f;
        g_rotate720_target_accum = ROT720_BASE_TURN;
        g_rotate720_dir = 1.0f;
        g_rotate720_target_valid = false;
        g_rotate720_final_target = final_target;
        g_rotate720_final_target_valid = final_target_valid;
        g_rotate720_lock_target = final_target;
        g_rotate720_brake_ticks = 0u;
        g_rotate720_brake_stable_ticks = 0u;
        g_rotate720_settle_ticks = 0u;
        g_rotate720_settle_stable_ticks = 0u;
    }
}

void track_rotate720_start(void)
{
    rotate720_start_common(false, 0.0f);
}

void track_rotate720_start_with_target(float target_direction)
{
    rotate720_start_common(true, target_direction);
}

bool track_rotate720_is_active(void)
{
    return (g_rotate720_state == ROTATE720_BRAKING ||
            g_rotate720_state == ROTATE720_ACTIVE ||
            g_rotate720_state == ROTATE720_SETTLING);
}

bool track_rotate720_is_done(void)
{
    return (g_rotate720_state == ROTATE720_DONE);
}

bool track_rotate720_should_suppress_odom(void)
{
    return (g_rotate720_state == ROTATE720_ACTIVE ||
            g_rotate720_state == ROTATE720_SETTLING);
}

void track_rotate720_reset(void)
{
    g_rotate720_state  = ROTATE720_IDLE;
    g_rotate720_accum  = 0.0f;
    g_rotate720_target = 0.0f;
    g_rotate720_target_accum = ROT720_BASE_TURN;
    g_rotate720_dir = 1.0f;
    g_rotate720_target_valid = false;
    g_rotate720_final_target = 0.0f;
    g_rotate720_final_target_valid = false;
    g_rotate720_lock_target = 0.0f;
    g_rotate720_brake_ticks = 0u;
    g_rotate720_brake_stable_ticks = 0u;
    g_rotate720_settle_ticks = 0u;
    g_rotate720_settle_stable_ticks = 0u;
}

void track_rotate720_update(Sensor_data_t *sensor, Move_cmd_t *cmd)
{
    if (g_rotate720_state != ROTATE720_BRAKING &&
        g_rotate720_state != ROTATE720_ACTIVE &&
        g_rotate720_state != ROTATE720_SETTLING) {
        return;
    }

    float path_target_direction = g_rotate720_final_target_valid ?
                                  g_rotate720_final_target :
                                  cmd->target_direction;

    cmd->target_speed = 0.0f;
    cmd->target_distance = 0.0f;

    if (!g_rotate720_target_valid) {
        g_rotate720_target = sensor->angle_yaw;
        g_rotate720_lock_target = rotate720_wrap_pi(path_target_direction);
        g_rotate720_target_valid = true;
    }

    if (g_rotate720_state == ROTATE720_BRAKING) {
        float body_speed_mps =
            0.5f * (sensor->motor_left_speed + sensor->motor_right_speed) *
            (LEG_WHEEL_RADIUS * 0.001f);

        cmd->target_direction = g_rotate720_target;
        cmd->target_roll = 0.0f;

        g_rotate720_brake_ticks++;
        if (fabsf(body_speed_mps) <= ROT720_BRAKE_SPEED_TOL_MPS) {
            g_rotate720_brake_stable_ticks++;
        } else {
            g_rotate720_brake_stable_ticks = 0u;
        }

        if (g_rotate720_brake_stable_ticks < ROT720_BRAKE_STABLE_TICKS &&
            g_rotate720_brake_ticks < ROT720_BRAKE_TIMEOUT_TICKS) {
            return;
        }

        g_rotate720_state = ROTATE720_ACTIVE;
        g_rotate720_accum = 0.0f;
        g_rotate720_target = sensor->angle_yaw;

        float final_delta = rotate720_wrap_pi(g_rotate720_lock_target -
                                              sensor->angle_yaw);
        g_rotate720_dir = (final_delta < -ROT720_SETTLE_YAW_TOL) ? -1.0f : 1.0f;
        g_rotate720_target_accum = ROT720_BASE_TURN + fabsf(final_delta);
    }

    /* 每周期递增目标角度, 不归一化 (yaw 误差归一化逻辑处理角度回绕) */
    if (g_rotate720_state == ROTATE720_ACTIVE) {
        float remaining = g_rotate720_target_accum - g_rotate720_accum;
        float target_rate = ROT720_SPEED;

        if (remaining < ROT720_SLOWDOWN_ANGLE) {
            target_rate = CLAMP(remaining * ROT720_SLOWDOWN_GAIN,
                                ROT720_MIN_SPEED,
                                ROT720_SPEED);
        }

        g_rotate720_target += g_rotate720_dir * target_rate * ROBOT_CONTROL_DT;
        cmd->target_direction = g_rotate720_target;

        /* 增强压弯: 旋转期间用更高系数 */
        float roll_from_turn = -ROT720_LEAN_GAIN * sensor->gyro_yaw / MAX_YAW_RATE;
        cmd->target_roll = CLAMP(roll_from_turn, -1.0f, 1.0f);

        /* 用实际 gyro 积分累积转角 (取绝对值, 方向由速率指令保证) */
        g_rotate720_accum += fabsf(sensor->gyro_yaw) * ROBOT_CONTROL_DT;

        if (g_rotate720_accum >= g_rotate720_target_accum + ROT720_MARGIN) {
            g_rotate720_state = ROTATE720_SETTLING;
            g_rotate720_settle_ticks = 0u;
            g_rotate720_settle_stable_ticks = 0u;
        }
    }

    if (g_rotate720_state == ROTATE720_SETTLING) {
        float yaw_error = rotate720_wrap_pi(g_rotate720_lock_target -
                                            sensor->angle_yaw);
        float roll_from_turn = -ROT720_SETTLE_LEAN_GAIN *
                               sensor->gyro_yaw / MAX_YAW_RATE;

        cmd->target_direction = g_rotate720_lock_target;
        cmd->target_roll = CLAMP(roll_from_turn, -1.0f, 1.0f);

        g_rotate720_settle_ticks++;
        if (fabsf(yaw_error) <= ROT720_SETTLE_YAW_TOL &&
            fabsf(sensor->gyro_yaw) <= ROT720_SETTLE_RATE_TOL) {
            g_rotate720_settle_stable_ticks++;
        } else {
            g_rotate720_settle_stable_ticks = 0u;
        }

        if (g_rotate720_settle_stable_ticks >= ROT720_SETTLE_STABLE_TICKS ||
            g_rotate720_settle_ticks >= ROT720_SETTLE_TIMEOUT_TICKS) {
            g_rotate720_state = ROTATE720_DONE;
        }
    }
}

/* =========================== 单边桥与爬坡 =========================== */

#define UPHILL_SPEED              -0.4f   /* 爬坡速度 */
#define UPHILL_TIMEOUT_MS         10000   /* 爬坡超时 */
#define SINGLE_BRIDGE_SPEED       -0.3f   /* 单边桥速度 */
#define SINGLE_BRIDGE_TIMEOUT_MS  4000    /* 单边桥超时 */
#define SINGLE_BRIDGE_ROLL_KP     -20.0f  /* 单边桥 roll PID KP, 保持车身水平 */

static bool     g_bridge_climb_active  = false;
static bool     g_bridge_use_roll      = false;  /* 单边桥: 启用 roll 闭环 */
static float    g_bridge_target_speed  = 0.0f;
static uint16_t g_bridge_timeout_ms    = 0;
static uint16_t g_bridge_climb_timer   = 0;

void track_bridge_climb_init(void)
{
    g_bridge_climb_active = false;
    g_bridge_use_roll     = false;
    g_bridge_climb_timer  = 0;
}

/* ── 爬坡: 无 roll, 速度 -0.4 ── */
void track_bridge_climb_activate(void)
{
    g_bridge_climb_active = true;
    g_bridge_use_roll     = false;
    g_bridge_target_speed = UPHILL_SPEED;
    g_bridge_timeout_ms   = UPHILL_TIMEOUT_MS;
    g_bridge_climb_timer  = 0;
}

/* ── 单边桥: 有 roll, 速度 -0.3 ── */
void track_single_bridge_activate(void)
{
    g_bridge_climb_active = true;
    g_bridge_use_roll     = true;
    g_bridge_target_speed = SINGLE_BRIDGE_SPEED;
    g_bridge_timeout_ms   = SINGLE_BRIDGE_TIMEOUT_MS;
    g_bridge_climb_timer  = 0;
    g_leg_roll_pid.kp     = SINGLE_BRIDGE_ROLL_KP;
    pid_reset(&g_leg_roll_pid);
}

void track_bridge_climb_deactivate(void)
{
    g_bridge_climb_active = false;
    g_bridge_climb_timer  = 0;
    if (g_bridge_use_roll) {
        g_leg_roll_pid.kp = 0.0f;
        g_bridge_use_roll = false;
    }
}

void track_single_bridge_deactivate(void)
{
    track_bridge_climb_deactivate();  /* 共用退出逻辑 (含 roll 恢复) */
}

bool track_bridge_climb_is_active(void)
{
    return g_bridge_climb_active;
}

void track_bridge_climb_apply(Move_cmd_t *cmd)
{
    if (g_bridge_climb_active) {
        cmd->target_speed = g_bridge_target_speed;
        g_bridge_climb_timer++;
        if (g_bridge_climb_timer >= g_bridge_timeout_ms) {
            track_bridge_climb_deactivate();
        }
    }
}

/* =========================== 颠簸路段 ===========================
 *  策略: 双腿伸腿(负Y)抬高底盘 + 平衡PID等比缩小 + 低速通过
 *  适用于密集路肩: 2cm高 / 2.5cm宽, 间距 ~10cm, 总长 >=1m
 *  倾角保护在 robot_control.c 中 conditionally 关闭 */

/* ── 辅助: 线性插值 ── */
static float bumpy_lerp(float from, float to, float t) {
    if (t <= 0.0f) return from;
    if (t >= 1.0f) return to;
    return from + (to - from) * t;
}

/* ── 参数 ── */
#define BUMPY_SPEED               -0.04f
#define BUMPY_LEG_KP              1000.0f  /* 腿稍软, 伸长后力矩放大 */
#define BUMPY_LEG_KD              3.0f
#define BUMPY_LEG_EXTEND          -160.0f  /* 双腿伸腿 (mm), 负=伸腿=提底盘 */
#define BUMPY_RAMP_MS             250      /* 伸腿渐变时间, 行程增大需延长 */
#define BUMPY_TIMEOUT_MS          10000    /* 总超时 10s */

/* 平衡 PID: 等效摆长 ~310mm, 缩放比 sqrt(150/310) ≈ 0.70 */
#define BUMPY_BALANCE_ANGLE_KP    17.0f    /* 标称 25.0 * 0.70 */
#define BUMPY_BALANCE_GYRO_KP     -750.0f  /* 标称 -1100 * 0.70 */
#define BUMPY_BALANCE_GYRO_KD    2.0f     /* 标称 3.0 * 0.70 */

/* 标称值 (用于恢复) */
#define NOMINAL_LEG_KP            1200.0f
#define NOMINAL_LEG_KD            4.0f
#define NOMINAL_BALANCE_ANGLE_KP  25.0f
#define NOMINAL_BALANCE_GYRO_KP   -1100.0f
#define NOMINAL_BALANCE_GYRO_KD  3.0f

/* ── 状态 ── */
static bool     g_bumpy_active     = false;
static float    g_bumpy_yaw_bias   = 0.0f;
static float    g_bumpy_left_lift  = 0.0f;
static float    g_bumpy_right_lift = 0.0f;
static bool     g_speed_inited     = false;
static uint16_t g_bumpy_timer      = 0;

/* ── 公开接口 ── */

void track_bumpy_init(void)
{
    g_bumpy_active     = false;
    g_bumpy_yaw_bias   = 0.0f;
    g_bumpy_left_lift  = 0.0f;
    g_bumpy_right_lift = 0.0f;
    g_speed_inited     = false;
}

void track_bumpy_activate(void)
{
    g_bumpy_active = true;
    g_speed_inited = false;
}

void track_bumpy_deactivate(void)
{
    g_bumpy_active = false;
}

bool track_bumpy_is_active(void)
{
    return g_bumpy_active;
}

float track_bumpy_get_speed(void)
{
    return BUMPY_SPEED;
}

float track_bumpy_get_yaw_bias(void)
{
    return g_bumpy_yaw_bias;
}

float track_bumpy_get_left_lift(void)
{
    return g_bumpy_left_lift;
}

float track_bumpy_get_right_lift(void)
{
    return g_bumpy_right_lift;
}

/* ── 增益切换 ── */

void track_bumpy_apply_compliance(void)
{
    g_leg_left_pid.front.kp  = BUMPY_LEG_KP;
    g_leg_left_pid.front.kd  = BUMPY_LEG_KD;
    g_leg_left_pid.back.kp   = BUMPY_LEG_KP;
    g_leg_left_pid.back.kd   = BUMPY_LEG_KD;
    g_leg_right_pid.front.kp = BUMPY_LEG_KP;
    g_leg_right_pid.front.kd = BUMPY_LEG_KD;
    g_leg_right_pid.back.kp  = BUMPY_LEG_KP;
    g_leg_right_pid.back.kd  = BUMPY_LEG_KD;
}

void track_bumpy_restore_stiffness(void)
{
    g_leg_left_pid.front.kp  = NOMINAL_LEG_KP;
    g_leg_left_pid.front.kd  = NOMINAL_LEG_KD;
    g_leg_left_pid.back.kp   = NOMINAL_LEG_KP;
    g_leg_left_pid.back.kd   = NOMINAL_LEG_KD;
    g_leg_right_pid.front.kp = NOMINAL_LEG_KP;
    g_leg_right_pid.front.kd = NOMINAL_LEG_KD;
    g_leg_right_pid.back.kp  = NOMINAL_LEG_KP;
    g_leg_right_pid.back.kd  = NOMINAL_LEG_KD;
}

void track_bumpy_apply_balance_pid(void)
{
    g_pitch_angle_pid.kp = BUMPY_BALANCE_ANGLE_KP;
    g_pitch_gyro_pid.kp  = BUMPY_BALANCE_GYRO_KP;
    g_pitch_gyro_pid.kd  = BUMPY_BALANCE_GYRO_KD;
}

void track_bumpy_restore_balance_pid(void)
{
    g_pitch_angle_pid.kp = NOMINAL_BALANCE_ANGLE_KP;
    g_pitch_gyro_pid.kp  = NOMINAL_BALANCE_GYRO_KP;
    g_pitch_gyro_pid.kd  = NOMINAL_BALANCE_GYRO_KD;
}

/* ── 状态机 ── */

void track_bumpy_update(const Sensor_data_t *sensor)
{
    if (!g_bumpy_active || sensor == NULL) return;

    if (!g_speed_inited) {
        g_speed_inited    = true;
        g_bumpy_timer     = 0;
        g_bumpy_yaw_bias  = 0.0f;
        g_bumpy_left_lift  = 0.0f;
        g_bumpy_right_lift = 0.0f;
        return;
    }

    g_bumpy_timer++;

    /* 总超时保护 */
    if (g_bumpy_timer >= BUMPY_TIMEOUT_MS) {
        g_bumpy_active     = false;
        g_bumpy_yaw_bias   = 0.0f;
        g_bumpy_left_lift  = 0.0f;
        g_bumpy_right_lift = 0.0f;
        return;
    }

    /* 直线行驶 */
    g_bumpy_yaw_bias = 0.0f;

    /* 双腿伸腿: 渐变 0 → LEG_EXTEND (负值 = 伸腿 = 提底盘) */
    if (g_bumpy_timer < BUMPY_RAMP_MS) {
        float t = (float)g_bumpy_timer / (float)BUMPY_RAMP_MS;
        float lift = bumpy_lerp(0.0f, BUMPY_LEG_EXTEND, t);
        g_bumpy_left_lift  = lift;
        g_bumpy_right_lift = lift;
    } else {
        g_bumpy_left_lift  = BUMPY_LEG_EXTEND;
        g_bumpy_right_lift = BUMPY_LEG_EXTEND;
    }
}
