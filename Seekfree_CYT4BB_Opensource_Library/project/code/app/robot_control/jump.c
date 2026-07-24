#include "jump.h"
#include "robot_control.h"
#include <math.h>
#include <stdbool.h>

/* =========================== 状态枚举 =========================== */
typedef enum {
    JUMP_IDLE = 0,
    JUMP_STABILIZE,     /* 自稳等待 (着地) */
    JUMP_SQUAT,         /* 下蹲蓄力 (着地) */
    JUMP_LAUNCH,        /* 蹬地伸腿 (着地, 至离地) */
    JUMP_AIR,           /* 空中: 收腿 → 伸腿够地 → 等触地 */
    JUMP_LAND           /* 缓冲着地 */
} JumpState_t;

/* ===================== 可调参数 ===================== */

/* Y 偏移 (mm, 叠加到 leg_cmd_solve 的 Y 上) */
#define JUMP_SQUAT_Y        100.0f   /* 下蹲收腿深度 */
#define JUMP_PUSH_Y        -400.0f   /* 蹬地伸腿 (饱和在关节限位) */
#define JUMP_TUCK_Y          0.0f   /* 空中收腿 */
#define JUMP_REACH_Y        -60.0f   /* 空中伸腿够地 */
#define JUMP_CUSHION_DELTA   110.0f   /* 落地相对缓冲深度 */

/* 前向推进 */
#define JUMP_FORWARD_BIAS   -0.0f   /* 蹬地时脚在髋后方的偏移 (mm) */

/* 空中惯量补偿: 足端前移对抗前重后轻 (mm) */
#define JUMP_AIR_X_COMPENSATE  80.0f

/* 时序 (1kHz cycles) */
#define STABILIZE_DURATION      1000    /* 单次起跳前自稳 1.0 秒 */
#define MULTI_STABILIZE_DURATION 125    /* 多级跳跃时每跳自稳 300ms (台阶面积有限, 不宜太长) */
#define JUMP23_STABILIZE_MS     100    /* 第二→第三跳之间自稳, 独立可调 */
#define SQUAT_DURATION          400
#define MULTI_SQUAT_DURATION    120     /* 多级跳跃时加速下蹲 */
#define LAUNCH_RAMP_MS         50    /* 伸腿渐变时间 */
#define LAUNCH_TIMEOUT        200
#define TUCK_RAMP_MS           30
#define REACH_RAMP_MS          30
#define CUSHION_RAMP_MS        30
#define CUSHION_HOLD_MS       200
#define MULTI_CUSHION_HOLD_MS   0   /* 多级跳跃时保持缓冲位让速度环稳定 */
#define MULTI_CUSHION_RELEASE_MS 60   /* 多级跳跃时渐进释放到中立位 */
#define CUSHION_RELEASE_MS    400
#define BALANCE_RAMP_MS        50

/* 飞行阶段检测阈值 */
#define IMPACT_ACCEL          1.2f    /* accel_z > 此值 = 触地 */
#define FREEZE_MS             15      /* 伸腿后冻结期, 防误触发 */
#define FLY_TIMEOUT_CYCLES    500     /* 飞行总超时 */
#define ACCEL_FREEFALL_THRESHOLD 0.3f /* 自由落体判据 */

/* 跳跃结束后速度环冷却期 (ms) */
#define JUMP_COOLDOWN_MS      500

/* 腿 PID 临时高增益 (LAUNCH 阶段减少力冲击持续时间) */
#define LAUNCH_KP             10000.0f
#define LAUNCH_KI             50.0f
#define NOMINAL_KP            1200.0f
#define NOMINAL_KI            0.0f

/* ======================== 静态变量 ============================= */

static JumpState_t  g_state    = JUMP_IDLE;
static uint16_t     g_cycle    = 0;
static float        g_approach_speed = 0.0f;  /* jump_start 传入的前向速度 */
static uint8_t      g_jump_remaining = 0;     /* 剩余跳跃次数 */
static uint8_t      g_jump_total     = 0;     /* 初始跳跃总次数, 用于区分首跳 */

/* LAND: 触地时记录的腿 Y 位置, 用于相对缓冲 */
static float g_land_y0 = 0.0f;

/* 离地瞬间捕获的足端 X (含重心补偿), 空中阶段保持 */
static float g_air_x_left  = 0.0f;
static float g_air_x_right = 0.0f;

/* SQUAT 开始时捕获的足端 Y 基线 (含 roll 补偿, 保留重心不对称) */
static float g_base_y_left  = 0.0f;
static float g_base_y_right = 0.0f;

/* 跳跃结束后冷却计时器 (cycles), 期间 g_speed_pid 保持关闭 */
static uint16_t g_cooldown_timer = 0;

/* ======================== 辅助函数 ============================= */

static float lerp(float from, float to, float t) {
    if (t <= 0.0f) return from;
    if (t >= 1.0f) return to;
    return from + (to - from) * t;
}

static void enter_state(JumpState_t new_state) {
    g_state = new_state;
    g_cycle = 0;
}

/** 临时拉高腿 PID 增益 (LAUNCH 阶段) */
static void leg_pid_set_launch_gains(void) {
    g_leg_left_pid.front.kp  = LAUNCH_KP;
    g_leg_left_pid.front.ki  = LAUNCH_KI;
    g_leg_left_pid.back.kp   = LAUNCH_KP;
    g_leg_left_pid.back.ki   = LAUNCH_KI;
    g_leg_right_pid.front.kp = LAUNCH_KP;
    g_leg_right_pid.front.ki = LAUNCH_KI;
    g_leg_right_pid.back.kp  = LAUNCH_KP;
    g_leg_right_pid.back.ki  = LAUNCH_KI;
}

/** 恢复腿 PID 标称增益 */
static void leg_pid_restore_nominal_gains(void) {
    g_leg_left_pid.front.kp  = NOMINAL_KP;
    g_leg_left_pid.front.ki  = NOMINAL_KI;
    g_leg_left_pid.back.kp   = NOMINAL_KP;
    g_leg_left_pid.back.ki   = NOMINAL_KI;
    g_leg_right_pid.front.kp = NOMINAL_KP;
    g_leg_right_pid.front.ki = NOMINAL_KI;
    g_leg_right_pid.back.kp  = NOMINAL_KP;
    g_leg_right_pid.back.ki  = NOMINAL_KI;
}

/* ====================== 状态机 ============================= */

/* ── STABILIZE: 自稳等待 (着地, 3秒) ──
 *   正常控制运行, 不做任何叠加, 纯计时等车身稳定.
 */
static void run_stabilize(const Sensor_data_t *sensor,
                          Foot_position_t *left, Foot_position_t *right)
{
    (void)sensor;

    if (g_cycle == 1) {
        robot_control_reset_leg_speed_pid();   /* 清除正常行驶的积分, 防止带入跳跃 (含 roll PID) */
        robot_control_reset_balance_pid();     /* 清除标定期间积累的平衡 PID 积分, 保证每次发车速度一致 */
    }

    /* 保留 leg_cmd_solve 的 X 输出, 让 g_leg_speed_pid 控制前向速度 */
    /* 清零 Y, 关闭 roll 闭环 */
    left->y  = 0;
    right->y = 0;

    /* 首跳用长自稳, 最后一跳用 JUMP23_STABILIZE_MS, 其他用 MULTI_STABILIZE_DURATION */
    uint16_t duration;
    if (g_jump_remaining == g_jump_total) {
        duration = STABILIZE_DURATION;
    } else if (g_jump_remaining == 1) {
        duration = JUMP23_STABILIZE_MS;
    } else {
        duration = MULTI_STABILIZE_DURATION;
    }

    if (g_cycle >= duration) {
        enter_state(JUMP_SQUAT);
    }
}

/* ── SQUAT: 下蹲蓄力 (着地, ~400ms) ──
 *   leg_cmd_solve 照常运行 (提供 X 基准).
 *   Y = 基线(含roll补偿) + 收腿偏移, 保留重心不对称.
 */
static void run_squat(const Sensor_data_t *sensor,
                      Foot_position_t *left, Foot_position_t *right)
{
    (void)sensor;

    if (g_cycle == 1) {
        g_base_y_left  = left->y;   /* 捕获 leg_cmd_solve 的 roll 补偿基线 */
        g_base_y_right = right->y;
    }

    bool is_multi = (g_jump_remaining != g_jump_total);
    uint16_t duration = is_multi ? MULTI_SQUAT_DURATION : SQUAT_DURATION;

    float t = (float)g_cycle / (float)duration;
    if (t > 1.0f) t = 1.0f;
    float dy = lerp(0.0f, JUMP_SQUAT_Y, t);

    /* 足端 X 居中, 避免蹲下时重心偏移 */
    left->x  = 0.0f;
    right->x = 0.0f;
    left->y  = g_base_y_left  + dy;
    right->y = g_base_y_right + dy;

    if (g_cycle >= duration) {
        enter_state(JUMP_LAUNCH);
    }
}

/* ── LAUNCH: 蹬地 (着地, max ~200ms) ──
 *   leg_cmd_solve 照常运行. 平衡照常运行.
 *   叠加: 腿 Y 从 squat 渐变到 PUSH (50ms 渐变, 给平衡环反应时间).
 *        腿 X 加对称前向偏置 JUMP_FORWARD_BIAS.
 *   腿 PID 临时拉高增益.
 *   退出: |accel_z| < 0.3g (离地) 或超时. 离地时复位平衡 PID.
 */
static void run_launch(const Sensor_data_t *sensor,
                       Foot_position_t *left, Foot_position_t *right)
{
    if (g_cycle == 1) {
        /* 纯前馈基于接近速度, 不依赖 PID, 多跳一致 */
        float ff_x = g_approach_speed * 200.0f;
        ff_x = CLAMP(ff_x, -80.0f, 80.0f);
        g_air_x_left  = ff_x;
        g_air_x_right = -ff_x;
        leg_pid_set_launch_gains();
    }

    /* 腿 Y: 50ms 渐变, 基线 + 蹬地偏移 */
    float ramp_t = (float)g_cycle / (float)LAUNCH_RAMP_MS;
    if (ramp_t > 1.0f) ramp_t = 1.0f;
    float dy = lerp(JUMP_SQUAT_Y, JUMP_PUSH_Y, ramp_t);

    left->x   = JUMP_FORWARD_BIAS;
    right->x  = -JUMP_FORWARD_BIAS;
    left->y  = g_base_y_left  + dy;
    right->y = g_base_y_right + dy;

    /* 离地检测: 单跳蹬地渐变结束后即启用, 防止腿在空中过度伸展 */
    bool airborne = false;
    uint16_t freefall_delay = 80;
    if (g_cycle > freefall_delay) {
        airborne = (fabsf(sensor->accel_z) < ACCEL_FREEFALL_THRESHOLD);
    }

    if (airborne || g_cycle >= LAUNCH_TIMEOUT) {
        leg_pid_restore_nominal_gains();
        robot_control_reset_balance_pid();
        robot_control_reset_leg_speed_pid();
        enter_state(JUMP_AIR);
    }
}

/* ── AIR: 空中阶段 (max ~500ms) ──
 *   airborne=true: 轮式平衡关停, leg_cmd_solve 跳过.
 *   子阶段 A (TUCK_RAMP_MS):  收腿 PUSH→TUCK (纯时序, 加速度计无法区分上升/下降)
 *   子阶段 B (至触地/超时):   伸腿 TUCK→REACH, 然后等 accel_z>1.2 触地冲击
 *   X 保持离地瞬间捕获的足端位置.
 */
static void run_air(const Sensor_data_t *sensor,
                    Foot_position_t *left, Foot_position_t *right)
{
    float y;
    float tuck_target = JUMP_TUCK_Y;
    uint16_t tuck_ms  = TUCK_RAMP_MS;
    uint16_t reach_ms = REACH_RAMP_MS;

    if (g_cycle < tuck_ms) {
        /* A: 收腿过障碍 */
        float t = (float)g_cycle / (float)tuck_ms;
        y = lerp(JUMP_PUSH_Y, tuck_target, t);
    } else if (g_cycle < (uint16_t)(tuck_ms + reach_ms)) {
        /* B: 伸腿够地 */
        uint16_t rel = g_cycle - tuck_ms;
        float t = (float)rel / (float)reach_ms;
        y = lerp(tuck_target, JUMP_REACH_Y, t);
    } else {
        /* C: 保持伸腿位, 等触地 */
        y = JUMP_REACH_Y;
    }

    float air_compensate = JUMP_AIR_X_COMPENSATE;
    left->x  = g_air_x_left  + air_compensate;
    right->x = g_air_x_right - air_compensate;
    left->y  = y;
    right->y = y;

    bool impact = (sensor->accel_z > IMPACT_ACCEL)
                  && (g_cycle > (uint16_t)(TUCK_RAMP_MS + REACH_RAMP_MS + FREEZE_MS));

    if (impact || g_cycle > FLY_TIMEOUT_CYCLES) {
        g_land_y0 = y;  /* 记录触地时的腿 Y */
        robot_control_reset_leg_speed_pid();
        enter_state(JUMP_LAND);
    }
}

/* ── LAND: 缓冲着地 (~630ms 单跳, ~280ms 多跳) ──
 *   airborne=false: 轮式平衡和 leg_cmd_solve 恢复运行.
 *   单跳: A(30ms) 缓冲 + B(200ms) 保持 + C(400ms) 释放 → IDLE + 冷却
 *   多跳: A(30ms) 缓冲 + B(150ms) 保持 + C(100ms) 释放 → SQUAT
 *   足端 X 居中, 不由 leg_cmd_solve 速度环控制.
 */
static void run_land(const Sensor_data_t *sensor,
                     Foot_position_t *left, Foot_position_t *right)
{
    (void)sensor;
    float y;
    bool is_multi = (g_jump_remaining > 1);

    /* 足端 X 居中, 落地时不产生额外水平力矩 */
    left->x  = 0;
    right->x = 0;

    if (is_multi) {
        uint16_t phase_a  = CUSHION_RAMP_MS;
        uint16_t phase_ab = phase_a + MULTI_CUSHION_HOLD_MS;
        uint16_t phase_abc = phase_ab + MULTI_CUSHION_RELEASE_MS;

        if (g_cycle <= phase_a) {
            float t = (float)g_cycle / (float)CUSHION_RAMP_MS;
            y = lerp(g_land_y0, g_land_y0 + JUMP_CUSHION_DELTA, t);
        } else if (g_cycle <= phase_ab) {
            y = g_land_y0 + JUMP_CUSHION_DELTA;
        } else {
            uint16_t rel = g_cycle - phase_ab;
            float t = (float)rel / (float)MULTI_CUSHION_RELEASE_MS;
            if (t > 1.0f) t = 1.0f;
            y = lerp(g_land_y0 + JUMP_CUSHION_DELTA, 0.0f, t);
        }

        left->y  = y;
        right->y = y;

        if (g_cycle >= phase_abc) {
            robot_control_reset_leg_speed_pid();
            g_jump_remaining--;
            /* 最后一跳前先进自稳, 给 JUMP23_STABILIZE_MS 单独调节窗口 */
            if (g_jump_remaining == 1) {
                enter_state(JUMP_STABILIZE);
            } else {
                enter_state(JUMP_SQUAT);
            }
        }
    } else {
        /* 落地 X: 固定 -50 */
        {
            float land_x = -50.0f;
            left->x  = land_x;
            right->x = -land_x;
        }

        uint16_t hold_ms  = CUSHION_HOLD_MS;
        uint16_t phase_ab = CUSHION_RAMP_MS + hold_ms;

        if (g_cycle <= CUSHION_RAMP_MS) {
            float t = (float)g_cycle / (float)CUSHION_RAMP_MS;
            y = lerp(g_land_y0, g_land_y0 + JUMP_CUSHION_DELTA, t);
        } else if (g_cycle <= phase_ab) {
            y = g_land_y0 + JUMP_CUSHION_DELTA;
        } else {
            uint16_t rel = g_cycle - phase_ab;
            float t = (float)rel / (float)CUSHION_RELEASE_MS;
            if (t > 1.0f) t = 1.0f;
            y = lerp(g_land_y0 + JUMP_CUSHION_DELTA, 0.0f, t);
        }

        left->y  = y;
        right->y = y;

        if (g_cycle >= phase_ab + CUSHION_RELEASE_MS) {
            robot_control_reset_leg_speed_pid();
            g_jump_remaining = 0;
            g_jump_total     = 0;
            g_cooldown_timer = JUMP_COOLDOWN_MS;  /* 最后一跳后关轮速环 */
            enter_state(JUMP_IDLE);
        }
    }
}

/* ======================== 公开接口 ============================= */

void jump_init(void) {
    g_state   = JUMP_IDLE;
    g_cycle   = 0;
    g_land_y0 = 0.0f;
    g_jump_remaining = 0;
    g_jump_total     = 0;
}

void jump_start(float target_speed, uint8_t count) {
    if (g_state == JUMP_IDLE && count > 0) {
        g_approach_speed = target_speed;
        g_jump_remaining = count;
        g_jump_total     = count;
        g_cooldown_timer = 0;  /* 清除冷却期, 允许立即起跳 */
        enter_state(JUMP_STABILIZE);
    }
}

void jump_stop(void) {
    if (g_state != JUMP_IDLE) {
        leg_pid_restore_nominal_gains();
        robot_control_reset_balance_pid();
        robot_control_reset_leg_speed_pid();
        g_jump_remaining = 0;
        g_jump_total     = 0;
        g_cooldown_timer = JUMP_COOLDOWN_MS;  /* 紧急停止也进入冷却期 */
        enter_state(JUMP_IDLE);
    }
}

bool jump_is_active(void) {
    return (g_state != JUMP_IDLE);
}

bool jump_is_airborne(void) {
    return (g_state == JUMP_AIR);
}

bool jump_is_launching(void) {
    return (g_state == JUMP_LAUNCH);
}

bool jump_is_stabilizing(void) {
    return (g_state == JUMP_STABILIZE);
}

bool jump_is_squatting(void) {
    return (g_state == JUMP_SQUAT);
}

bool jump_is_in_cooldown(void) {
    return (g_cooldown_timer > 0);
}

uint8_t jump_get_remaining(void) {
    return g_jump_remaining;
}

float jump_get_approach_speed(void) {
    return g_approach_speed;
}

void jump_leg_overlay(Foot_position_t *left, Foot_position_t *right,
                      const Sensor_data_t *sensor) {
    g_cycle++;

    switch (g_state) {
    case JUMP_STABILIZE:   run_stabilize(sensor, left, right);   break;
    case JUMP_SQUAT:       run_squat(sensor, left, right);       break;
    case JUMP_LAUNCH:      run_launch(sensor, left, right);      break;
    case JUMP_AIR:         run_air(sensor, left, right);         break;
    case JUMP_LAND:        run_land(sensor, left, right);        break;
    case JUMP_IDLE:
    default:
        if (g_cooldown_timer > 0) {
            g_cooldown_timer--;
        }
        break;
    }
}
