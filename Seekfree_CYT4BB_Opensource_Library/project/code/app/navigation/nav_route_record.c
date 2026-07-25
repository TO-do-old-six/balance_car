#include "nav_route_record.h"
#include "nav_internal.h"
#include "nav_route_storage.h"

#include <math.h>
#include <stddef.h>

#include "../../common/utils.h"
#include "../../hmi/indicator/led_buzzer.h"

#define NAV_RECORD_STRAIGHT_SPEED  (-0.25f)  /* 正常回放速度；绝对值越大直线路段越快 */
#define NAV_RECORD_CORNER_SPEED    (-0.18f) /* 航向误差较大时的低速；调小绝对值可更稳过弯 */
#define NAV_RECORD_TURN_SPEED      0.0f     /* 大角度偏航时的速度；0 表示先停住再修正航向 */
#define NAV_RECORD_WAYPOINT_REACHED_M          0.03f  /* 普通路径点到达半径(m)；调大更容易切到下一点 */
#define NAV_RECORD_FINAL_BRAKE_SPEED           0.2f   /* 终点制动速度指令；调大制动更强 */
#define NAV_RECORD_FINAL_BRAKE_TIME_MS         260u   /* 终点制动持续时间(ms)；调大停车拖得更久 */
#define NAV_RECORD_WAYPOINT_PASS_CROSSTRACK_M  0.04f  /* 越过普通路径点允许的横向误差(m)；调大更不易漏点 */
#define NAV_RECORD_SHORT_SEGMENT_M             0.015f /* 小于该距离的路径段视为短段(m)；调大可忽略更近的重复点 */
#define NAV_RECORD_CROSSTRACK_GAIN             2.0f   /* 横向偏差转航向修正的增益；调大回线更快但可能摆动 */
#define NAV_RECORD_CROSSTRACK_LIMIT_RAD        (20.0f * NAV_DEG_TO_RAD) /* 横向纠偏最大航向角(rad)；调大允许更猛回线 */
#define NAV_RECORD_SLOW_YAW_ERROR_RAD          (10.0f * NAV_DEG_TO_RAD) /* 超过该航向误差后降到过弯速度(rad)；调小会更早减速 */
#define NAV_RECORD_TURN_IN_PLACE_YAW_RAD       (25.0f * NAV_DEG_TO_RAD) /* 超过该航向误差后停车修正(rad)；调小会更早停下修正 */
#define NAV_RECORD_TURN_BRAKE_RELEASE_MPS      0.18f  /* 普通过弯强制制动释放速度(m/s)；调大更早退出制动 */
#define NAV_RECORD_TURN_BRAKE_SPEED            0.18f  /* 普通过弯强制制动指令；调大减速更猛 */
#define NAV_RECORD_TURN_BRAKE_DECEL_MPS2       0.80f  /* 普通过弯制动距离估算减速度(m/s^2)；调大预估制动距离更短 */
#define NAV_RECORD_TURN_BRAKE_MARGIN_M         0.18f  /* 普通过弯额外制动余量(m)；调大更早刹车 */
#define NAV_RECORD_TURN_BRAKE_MAX_LOOKAHEAD_M  1.00f  /* 普通过弯最大预瞄制动距离(m)；限制最早刹车点 */
#define NAV_RECORD_TURN_PREBRAKE_LOOKAHEAD_M   0.90f  /* 普通过弯渐进减速预瞄距离(m)；调大更早开始降速 */
#define NAV_RECORD_TURN_PREBRAKE_MIN_SEGMENT_M 0.80f  /* 启用普通过弯预减速的最短路段(m)；调小短段也会提前减速 */
#define NAV_RECORD_TURN_PREBRAKE_YAW_RAD       (45.0f * NAV_DEG_TO_RAD) /* 判定为大弯的夹角阈值(rad)；调小更多弯会触发预减速 */
#define NAV_RECORD_TURN_PREBRAKE_SPEED         (-0.06f) /* 普通过弯预减速目标速度；绝对值越小弯前越慢 */
#define NAV_RECORD_ROTATE_WAYPOINT_REACHED_M   0.01f  /* 旋转点到达半径(m)；调大更早进入旋转动作 */
#define NAV_RECORD_ROTATE_BRAKE_DECEL_MPS2     0.80f  /* 旋转点制动距离估算减速度(m/s^2)；调大预估制动距离更短 */
#define NAV_RECORD_ROTATE_BRAKE_MARGIN_M       0.65f  /* 旋转点额外制动余量(m)；调大更早刹到旋转点 */
#define NAV_RECORD_ROTATE_PREBRAKE_MIN_SEGMENT_M 0.05f /* 启用旋转点预制动的最短路段(m)；调小更容易触发 */
#define NAV_RECORD_ROTATE_PREBRAKE_DISTANCE_M  0.80f  /* 旋转点渐进减速开始距离(m)；调大更早慢下来 */
#define NAV_RECORD_ROTATE_CRAWL_DISTANCE_M     NAV_RECORD_ROTATE_PREBRAKE_DISTANCE_M /* 旋转点爬行距离(m)；进入预减速区后直接以爬行速度找点 */
#define NAV_RECORD_ROTATE_HARD_BRAKE_DISTANCE_M 0.035f /* 旋转点硬刹触发距离(m)；调大更早强制刹停 */
#define NAV_RECORD_ROTATE_CRAWL_SPEED          (-0.05f) /* 旋转点爬行速度；绝对值越小靠点越慢 */
#define NAV_RECORD_ROTATE_PASS_CROSSTRACK_M    0.015f /* 越过旋转点允许的横向误差(m)；调大更容易判定已越过 */
#define NAV_RECORD_ROTATE_HARD_BRAKE_SPEED     0.40f  /* 旋转点硬刹速度指令；调大刹停更快 */
#define NAV_RECORD_ROTATE_BRAKE_RELEASE_MPS    0.08f  /* 旋转点硬刹释放速度(m/s)；调大更早结束硬刹 */
#define NAV_RECORD_ROTATE_TRIGGER_SPEED_MPS    0.12f  /* 允许触发旋转动作的最高速度(m/s)；调大可不必完全停稳 */
#define NAV_RECORD_ROTATE_TRIGGER_DISTANCE_M   0.05f  /* 允许触发旋转动作的距离(m)；调大更早执行旋转 */

static Nav_Keypoint_t g_record_keypoints[NAV_RECORD_MAX_KEYPOINTS];
static Nav_Route_Record_State_t g_record_state;
static float g_record_origin_x;
static float g_record_origin_y;
static float g_record_origin_distance;
static float g_record_origin_yaw;
static float g_record_last_yaw;
static float g_record_yaw_accum;
static uint32 g_record_origin_time;
static float g_replay_start_x;
static float g_replay_start_y;
static float g_replay_start_yaw;
static bool g_replay_final_braking;
static uint32 g_replay_final_brake_start_time;
static float g_replay_final_brake_yaw;
static uint8 g_replay_brake_segment_index;
static bool g_replay_segment_brake_active;
static bool g_replay_segment_brake_done;

static void replay_reset_segment_brake(void)
{
    g_replay_brake_segment_index = g_record_state.replay_index;
    g_replay_segment_brake_active = false;
    g_replay_segment_brake_done = false;
}

static void replay_sync_segment_brake(void)
{
    if (g_replay_brake_segment_index != g_record_state.replay_index) {
        replay_reset_segment_brake();
    }
}

static void record_set_origin(const Nav_Input_t *input)
{
    g_record_origin_x = input->x_m;
    g_record_origin_y = input->y_m;
    g_record_origin_distance = input->distance_m;
    g_record_origin_yaw = input->yaw_rad;
    g_record_last_yaw = input->yaw_rad;
    g_record_yaw_accum = 0.0f;
    g_record_origin_time = input->time_ms;
}

static void record_keypoint_from_input(uint8 index, const Nav_Input_t *input)
{
    float dx = input->x_m - g_record_origin_x;
    float dy = input->y_m - g_record_origin_y;
    float cos_yaw = cosf(g_record_origin_yaw);
    float sin_yaw = sinf(g_record_origin_yaw);
    float relative_distance = input->distance_m - g_record_origin_distance;

    if (index == 0u) {
        g_record_yaw_accum = 0.0f;
    } else {
        g_record_yaw_accum += nav_wrap_pi(input->yaw_rad - g_record_last_yaw);
    }
    g_record_last_yaw = input->yaw_rad;

    g_record_keypoints[index].x_m = cos_yaw * dx + sin_yaw * dy;
    g_record_keypoints[index].y_m = -sin_yaw * dx + cos_yaw * dy;
    g_record_keypoints[index].distance_m =
        relative_distance > 0.0f ? relative_distance : 0.0f;
    g_record_keypoints[index].yaw_rad = g_record_yaw_accum;
    g_record_keypoints[index].time_ms =
        (uint32)(input->time_ms - g_record_origin_time);
    g_record_keypoints[index].action = NAV_ROUTE_POINT_ACTION_NONE;
}

static float keypoint_yaw_delta_from_start(uint8 index)
{
    if (g_record_state.keypoint_count == 0u) {
        return 0.0f;
    }

    if (index >= g_record_state.keypoint_count) {
        index = (uint8)(g_record_state.keypoint_count - 1u);
    }

    return g_record_keypoints[index].yaw_rad;
}

static void replay_transform_keypoint(uint8 index,
                                      float *x_m,
                                      float *y_m,
                                      float *yaw_rad)
{
    const Nav_Keypoint_t *keypoint;
    float cos_yaw;
    float sin_yaw;
    float dx;
    float dy;

    if (g_record_state.keypoint_count == 0u) {
        if (x_m != NULL) {
            *x_m = g_replay_start_x;
        }
        if (y_m != NULL) {
            *y_m = g_replay_start_y;
        }
        if (yaw_rad != NULL) {
            *yaw_rad = g_replay_start_yaw;
        }
        return;
    }

    if (index >= g_record_state.keypoint_count) {
        index = (uint8)(g_record_state.keypoint_count - 1u);
    }

    keypoint = &g_record_keypoints[index];
    cos_yaw = cosf(g_replay_start_yaw);
    sin_yaw = sinf(g_replay_start_yaw);
    dx = keypoint->x_m;
    dy = keypoint->y_m;

    if (x_m != NULL) {
        *x_m = g_replay_start_x + cos_yaw * dx - sin_yaw * dy;
    }
    if (y_m != NULL) {
        *y_m = g_replay_start_y + sin_yaw * dx + cos_yaw * dy;
    }
    if (yaw_rad != NULL) {
        *yaw_rad = nav_wrap_pi(g_replay_start_yaw +
                               keypoint_yaw_delta_from_start(index));
    }
}

static bool waypoint_passed_with_limit(float prev_x,
                                       float prev_y,
                                       float target_x,
                                       float target_y,
                                       float current_x,
                                       float current_y,
                                       float cross_track_limit)
{
    float vx = target_x - prev_x;
    float vy = target_y - prev_y;
    float wx = current_x - prev_x;
    float wy = current_y - prev_y;
    float segment_len_sq = vx * vx + vy * vy;
    float cross_track;

    if (segment_len_sq <= 0.000001f) {
        return false;
    }

    if ((vx * wx + vy * wy) < segment_len_sq) {
        return false;
    }

    cross_track = fabsf(vx * wy - vy * wx) / sqrtf(segment_len_sq);
    return cross_track <= cross_track_limit;
}

static bool waypoint_passed(float prev_x,
                            float prev_y,
                            float target_x,
                            float target_y,
                            float current_x,
                            float current_y)
{
    return waypoint_passed_with_limit(prev_x,
                                      prev_y,
                                      target_x,
                                      target_y,
                                      current_x,
                                      current_y,
                                      NAV_RECORD_WAYPOINT_PASS_CROSSTRACK_M);
}

static void replay_advance_waypoint(void)
{
    g_record_state.replay_index++;
    replay_reset_segment_brake();
    buzzer_beep(BEEP_SHORT);
}

static void replay_fill_waypoint_event(Nav_Output_t *out, uint8 index)
{
    if (out == NULL || index >= g_record_state.keypoint_count) {
        return;
    }

    out->waypoint_entered = true;
    out->waypoint_index = index;
    out->waypoint_action = g_record_keypoints[index].action;
}

static void replay_reset_final_brake(void)
{
    g_replay_final_braking = false;
    g_replay_final_brake_start_time = 0u;
    g_replay_final_brake_yaw = 0.0f;
}

static float replay_body_yaw_from_path_yaw(float path_yaw)
{
    return nav_wrap_pi(path_yaw + NAV_PI);
}

static bool replay_next_segment_body_yaw(uint8 index, float *yaw_rad)
{
    float from_x;
    float from_y;
    uint8 next_index;

    if (yaw_rad == NULL || index >= g_record_state.keypoint_count) {
        return false;
    }

    replay_transform_keypoint(index, &from_x, &from_y, NULL);

    for (next_index = (uint8)(index + 1u);
         next_index < g_record_state.keypoint_count;
         next_index++) {
        float to_x;
        float to_y;
        float dx;
        float dy;
        float distance;

        replay_transform_keypoint(next_index, &to_x, &to_y, NULL);
        dx = to_x - from_x;
        dy = to_y - from_y;
        distance = sqrtf(dx * dx + dy * dy);
        if (distance >= NAV_RECORD_SHORT_SEGMENT_M) {
            *yaw_rad = replay_body_yaw_from_path_yaw(atan2f(dy, dx));
            return true;
        }
    }

    return false;
}

static void replay_hold_current_yaw(Nav_Output_t *out, const Nav_Input_t *input)
{
    if (out == NULL || input == NULL) {
        return;
    }

    out->velocity_cmd = 0.0f;
    out->target_yaw_valid = true;
    out->target_yaw_rad = input->yaw_rad;
}

static Nav_Output_t replay_rotate_action_brake_update(const Nav_Input_t *input,
                                                       uint8 index)
{
    Nav_Output_t out = {0};
    float next_yaw_rad;

    if (input == NULL) {
        return out;
    }

    replay_fill_waypoint_event(&out, index);
    replay_advance_waypoint();
    if (replay_next_segment_body_yaw(index, &next_yaw_rad)) {
        out.velocity_cmd = 0.0f;
        out.target_yaw_valid = true;
        out.target_yaw_rad = next_yaw_rad;
    } else {
        replay_hold_current_yaw(&out, input);
    }
    return out;
}

static float replay_rotate_brake_distance(float speed_mps);

static bool replay_apply_rotate_prebrake(Nav_Output_t *out,
                                         const Nav_Input_t *input,
                                         float segment_distance,
                                         uint8 target_index,
                                         float target_distance)
{
    float ratio;
    float speed_range;
    float brake_distance;

    if (out == NULL || input == NULL ||
        target_index >= g_record_state.keypoint_count ||
        g_record_keypoints[target_index].action !=
        NAV_ROUTE_POINT_ACTION_ROTATE720) {
        return false;
    }

    if (g_replay_segment_brake_active) {
        if (fabsf(input->speed_mps) <= NAV_RECORD_ROTATE_BRAKE_RELEASE_MPS) {
            g_replay_segment_brake_active = false;
            g_replay_segment_brake_done = true;
        } else {
            out->velocity_cmd = NAV_RECORD_ROTATE_HARD_BRAKE_SPEED;
            return true;
        }
    }

    brake_distance = replay_rotate_brake_distance(input->speed_mps);
    if (!g_replay_segment_brake_done &&
        segment_distance >= NAV_RECORD_ROTATE_PREBRAKE_MIN_SEGMENT_M &&
        fabsf(input->speed_mps) > NAV_RECORD_ROTATE_BRAKE_RELEASE_MPS &&
        target_distance <= brake_distance) {
        g_replay_segment_brake_active = true;
        out->velocity_cmd = NAV_RECORD_ROTATE_HARD_BRAKE_SPEED;
        return true;
    }

    if (segment_distance >= NAV_RECORD_ROTATE_PREBRAKE_MIN_SEGMENT_M &&
        target_distance < NAV_RECORD_TURN_PREBRAKE_LOOKAHEAD_M) {
        ratio = 1.0f -
            clamp(target_distance / NAV_RECORD_TURN_PREBRAKE_LOOKAHEAD_M,
                  0.0f,
                  1.0f);

        out->velocity_cmd += (NAV_RECORD_TURN_PREBRAKE_SPEED -
                              out->velocity_cmd) * ratio;
    }

    if (target_distance <= NAV_RECORD_ROTATE_HARD_BRAKE_DISTANCE_M) {
        if (!g_replay_segment_brake_done) {
            g_replay_segment_brake_active = true;
            out->velocity_cmd = NAV_RECORD_ROTATE_HARD_BRAKE_SPEED;
            return true;
        }
    }

    if (target_distance <= NAV_RECORD_ROTATE_CRAWL_DISTANCE_M) {
        out->velocity_cmd = NAV_RECORD_ROTATE_CRAWL_SPEED;
        return false;
    }

    if (target_distance >= NAV_RECORD_ROTATE_PREBRAKE_DISTANCE_M) {
        return false;
    }

    speed_range = NAV_RECORD_ROTATE_PREBRAKE_DISTANCE_M -
                  NAV_RECORD_ROTATE_CRAWL_DISTANCE_M;
    if (speed_range <= 0.0f) {
        out->velocity_cmd = NAV_RECORD_ROTATE_CRAWL_SPEED;
        return false;
    }

    ratio = (target_distance - NAV_RECORD_ROTATE_CRAWL_DISTANCE_M) /
            speed_range;
    ratio = clamp(ratio, 0.0f, 1.0f);
    out->velocity_cmd = NAV_RECORD_ROTATE_CRAWL_SPEED +
        (out->velocity_cmd - NAV_RECORD_ROTATE_CRAWL_SPEED) * ratio;
    return false;
}

static float replay_upcoming_turn_angle(uint8 target_index,
                                        float segment_yaw)
{
    float corner_x;
    float corner_y;
    float next_x;
    float next_y;
    float next_dx;
    float next_dy;
    float next_distance;
    float next_yaw;

    if (target_index + 1u >= g_record_state.keypoint_count) {
        return 0.0f;
    }

    replay_transform_keypoint(target_index, &corner_x, &corner_y, NULL);
    replay_transform_keypoint((uint8)(target_index + 1u),
                              &next_x,
                              &next_y,
                              NULL);

    next_dx = next_x - corner_x;
    next_dy = next_y - corner_y;
    next_distance = sqrtf(next_dx * next_dx + next_dy * next_dy);
    if (next_distance < NAV_RECORD_SHORT_SEGMENT_M) {
        return 0.0f;
    }

    next_yaw = atan2f(next_dy, next_dx);
    return fabsf(nav_wrap_pi(next_yaw - segment_yaw));
}

static float replay_turn_brake_distance(float speed_mps)
{
    float speed_abs = fabsf(speed_mps);
    float brake_distance =
        speed_abs * speed_abs / (2.0f * NAV_RECORD_TURN_BRAKE_DECEL_MPS2);

    brake_distance += NAV_RECORD_TURN_BRAKE_MARGIN_M;
    return clamp(brake_distance,
                 0.0f,
                 NAV_RECORD_TURN_BRAKE_MAX_LOOKAHEAD_M);
}

static float replay_rotate_brake_distance(float speed_mps)
{
    float speed_abs = fabsf(speed_mps);
    float brake_distance =
        speed_abs * speed_abs / (2.0f * NAV_RECORD_ROTATE_BRAKE_DECEL_MPS2);

    brake_distance += NAV_RECORD_ROTATE_BRAKE_MARGIN_M;
    return clamp(brake_distance,
                 0.0f,
                 NAV_RECORD_TURN_BRAKE_MAX_LOOKAHEAD_M);
}

static bool replay_apply_turn_prebrake(Nav_Output_t *out,
                                       const Nav_Input_t *input,
                                       float segment_distance,
                                       float target_distance,
                                       float upcoming_turn_rad)
{
    float distance_ratio;
    float brake_distance;

    if (out == NULL ||
        input == NULL ||
        segment_distance < NAV_RECORD_TURN_PREBRAKE_MIN_SEGMENT_M ||
        upcoming_turn_rad < NAV_RECORD_TURN_PREBRAKE_YAW_RAD ||
        out->velocity_cmd >= 0.0f) {
        return false;
    }

    if (g_replay_segment_brake_active) {
        if (fabsf(input->speed_mps) <= NAV_RECORD_TURN_BRAKE_RELEASE_MPS) {
            g_replay_segment_brake_active = false;
            g_replay_segment_brake_done = true;
        } else {
            out->velocity_cmd = NAV_RECORD_TURN_BRAKE_SPEED;
            return true;
        }
    }

    brake_distance = replay_turn_brake_distance(input->speed_mps);
    if (!g_replay_segment_brake_done &&
        fabsf(input->speed_mps) > NAV_RECORD_TURN_BRAKE_RELEASE_MPS &&
        target_distance <= brake_distance) {
        g_replay_segment_brake_active = true;
        out->velocity_cmd = NAV_RECORD_TURN_BRAKE_SPEED;
        return true;
    }

    if (target_distance >= NAV_RECORD_TURN_PREBRAKE_LOOKAHEAD_M) {
        return false;
    }

    distance_ratio = 1.0f -
        clamp(target_distance / NAV_RECORD_TURN_PREBRAKE_LOOKAHEAD_M,
              0.0f,
              1.0f);

    out->velocity_cmd += (NAV_RECORD_TURN_PREBRAKE_SPEED -
                          out->velocity_cmd) * distance_ratio;
    return false;
}

static Nav_Output_t replay_final_brake_update(const Nav_Input_t *input)
{
    Nav_Output_t out = {0};
    uint32 elapsed_ms;

    if (input == NULL) {
        return out;
    }

    elapsed_ms = input->time_ms - g_replay_final_brake_start_time;
    if (elapsed_ms >= NAV_RECORD_FINAL_BRAKE_TIME_MS) {
        replay_reset_final_brake();
        g_record_state.mode = NAV_ROUTE_READY;
        g_record_state.replay_index = 0u;
        replay_reset_segment_brake();
        out.finished = true;
        replay_hold_current_yaw(&out, input);
        return out;
    }

    out.velocity_cmd = NAV_RECORD_FINAL_BRAKE_SPEED;
    out.target_yaw_valid = true;
    out.target_yaw_rad = g_replay_final_brake_yaw;
    out.region = NAV_REGION_NORMAL;
    return out;
}

void nav_route_record_notify_navigation_stopped(bool finished,
                                                bool safety_stop)
{
    if (g_record_state.mode == NAV_ROUTE_REPLAYING &&
        (finished || safety_stop)) {
        g_record_state.mode = g_record_state.route_ready ?
            NAV_ROUTE_READY : NAV_ROUTE_IDLE;
    }
}

bool nav_route_record_start(const Nav_Input_t *input)
{
    if (input == NULL) {
        return false;
    }

    nav_stop();
    g_record_state.mode = NAV_ROUTE_RECORDING;
    g_record_state.keypoint_count = 1u;
    g_record_state.replay_index = 0u;
    g_record_state.route_ready = false;
    g_record_state.overflow = false;
    replay_reset_final_brake();
    replay_reset_segment_brake();

    record_set_origin(input);
    record_keypoint_from_input(0u, input);

    return true;
}

bool nav_route_record_keypoint(const Nav_Input_t *input)
{
    return nav_route_record_keypoint_with_action(input,
                                                 NAV_ROUTE_POINT_ACTION_NONE);
}

bool nav_route_record_keypoint_with_action(const Nav_Input_t *input,
                                           Nav_Route_Point_Action_t action)
{
    if (input == NULL || g_record_state.mode != NAV_ROUTE_RECORDING) {
        return false;
    }

    if (g_record_state.keypoint_count >= NAV_RECORD_MAX_KEYPOINTS) {
        g_record_state.overflow = true;
        return false;
    }

    uint8 index = g_record_state.keypoint_count;
    record_keypoint_from_input(index, input);
    g_record_keypoints[index].action = action;
    g_record_state.keypoint_count++;

    return true;
}

bool nav_route_record_finish(void)
{
    bool saved;

    if (g_record_state.mode != NAV_ROUTE_RECORDING) {
        return false;
    }

    if (g_record_state.keypoint_count < 2u) {
        g_record_state.mode = NAV_ROUTE_IDLE;
        g_record_state.route_ready = false;
        return false;
    }

    saved = nav_route_storage_save(g_record_keypoints,
                                   g_record_state.keypoint_count);

    g_record_state.mode = NAV_ROUTE_READY;
    g_record_state.route_ready = true;
    g_record_state.replay_index = 0u;
    return saved;
}

bool nav_route_record_save_reserved_slot(uint8 reserved_slot)
{
    if (g_record_state.keypoint_count < 2u) {
        return false;
    }

    return nav_route_storage_save_reserved_slot(g_record_keypoints,
                                                g_record_state.keypoint_count,
                                                reserved_slot);
}

bool nav_route_record_save_reserved(void)
{
    return nav_route_record_save_reserved_slot(0u);
}

void nav_route_record_reset(void)
{
    nav_stop();
    g_record_state.mode = NAV_ROUTE_IDLE;
    g_record_state.keypoint_count = 0u;
    g_record_state.replay_index = 0u;
    g_record_state.route_ready = false;
    g_record_state.overflow = false;
    replay_reset_final_brake();
    replay_reset_segment_brake();
}

static bool set_loaded_keypoints_ready(uint8 keypoint_count)
{
    nav_stop();
    g_record_state.mode = NAV_ROUTE_IDLE;
    g_record_state.keypoint_count = keypoint_count;
    g_record_state.replay_index = 0u;
    g_record_state.route_ready = false;
    g_record_state.overflow = false;
    replay_reset_final_brake();
    replay_reset_segment_brake();

    if (g_record_state.keypoint_count < 2u) {
        g_record_state.keypoint_count = 0u;
        return false;
    }

    g_record_state.mode = NAV_ROUTE_READY;
    g_record_state.route_ready = true;
    return true;
}

bool nav_route_record_load_saved_history(uint8 history_index)
{
    uint8 keypoint_count = 0u;

    if (!nav_route_storage_load_history(g_record_keypoints,
                                        NAV_RECORD_MAX_KEYPOINTS,
                                        &keypoint_count,
                                        history_index)) {
        return false;
    }

    return set_loaded_keypoints_ready(keypoint_count);
}

bool nav_route_record_load_saved(void)
{
    return nav_route_record_load_saved_history(0u);
}

bool nav_route_record_load_previous_saved(void)
{
    return nav_route_record_load_saved_history(1u);
}

bool nav_route_record_load_reserved_slot(uint8 reserved_slot)
{
    uint8 keypoint_count = 0u;

    if (!nav_route_storage_load_reserved_slot(g_record_keypoints,
                                              NAV_RECORD_MAX_KEYPOINTS,
                                              &keypoint_count,
                                              reserved_slot)) {
        return false;
    }

    return set_loaded_keypoints_ready(keypoint_count);
}

bool nav_route_replay_start(const Nav_Input_t *input)
{
    if (input == NULL || !g_record_state.route_ready ||
        g_record_state.keypoint_count < 2u) {
        return false;
    }

    nav_stop();
    g_record_state.mode = NAV_ROUTE_REPLAYING;
    g_record_state.replay_index = 1u;
    g_replay_start_x = input->x_m;
    g_replay_start_y = input->y_m;
    g_replay_start_yaw = input->yaw_rad;
    replay_reset_final_brake();
    replay_reset_segment_brake();

    return true;
}

bool nav_route_replay_anchor_to_next_action(const Nav_Input_t *input,
                                            Nav_Route_Point_Action_t action)
{
    if (input == NULL ||
        action == NAV_ROUTE_POINT_ACTION_NONE ||
        g_record_state.mode != NAV_ROUTE_REPLAYING ||
        !g_record_state.route_ready ||
        g_record_state.keypoint_count < 2u) {
        return false;
    }

    for (uint8 index = g_record_state.replay_index;
         index < g_record_state.keypoint_count;
         index++) {
        if (g_record_keypoints[index].action == action) {
            const Nav_Keypoint_t *keypoint = &g_record_keypoints[index];
            float local_yaw = keypoint_yaw_delta_from_start(index);
            float start_yaw = nav_wrap_pi(input->yaw_rad - local_yaw);
            float cos_yaw = cosf(start_yaw);
            float sin_yaw = sinf(start_yaw);

            g_replay_start_yaw = start_yaw;
            g_replay_start_x = input->x_m -
                               cos_yaw * keypoint->x_m +
                               sin_yaw * keypoint->y_m;
            g_replay_start_y = input->y_m -
                               sin_yaw * keypoint->x_m -
                               cos_yaw * keypoint->y_m;
            g_record_state.replay_index = (uint8)(index + 1u);
            replay_reset_final_brake();
            replay_reset_segment_brake();
            return true;
        }
    }

    return false;
}

Nav_Output_t nav_route_replay_update(const Nav_Input_t *input)
{
    Nav_Output_t out = {0};
    Nav_Config_t cfg;
    float reach_radius;

    if (input == NULL ||
        g_record_state.mode != NAV_ROUTE_REPLAYING ||
        !g_record_state.route_ready ||
        g_record_state.keypoint_count < 2u) {
        return out;
    }

    if (input->obstacle_close) {
        g_record_state.mode = NAV_ROUTE_READY;
        g_record_state.replay_index = 0u;
        replay_reset_final_brake();
        replay_reset_segment_brake();
        out.safety_stop = true;
        replay_hold_current_yaw(&out, input);
        return out;
    }

    if (g_replay_final_braking) {
        return replay_final_brake_update(input);
    }

    cfg = nav_get_config();
    reach_radius = cfg.distance_tolerance_m;
    if (reach_radius <= 0.0f ||
        reach_radius > NAV_RECORD_WAYPOINT_REACHED_M) {
        reach_radius = NAV_RECORD_WAYPOINT_REACHED_M;
    }

    while (g_record_state.replay_index < g_record_state.keypoint_count) {
        uint8 prev_index = (uint8)(g_record_state.replay_index - 1u);
        uint8 cur_index = g_record_state.replay_index;
        float prev_x;
        float prev_y;
        float target_x;
        float target_y;
        float target_yaw_cmd;
        float segment_dx;
        float segment_dy;
        float segment_distance;
        float target_dx;
        float target_dy;
        float target_distance;
        float along_remaining;
        float waypoint_reach_radius;
        float segment_yaw;
        float cross_track_error;
        float yaw_correction;
        float yaw_error;
        float abs_yaw_error;
        float upcoming_turn_rad;
        bool braking_before_turn;
        bool braking_before_rotate;
        bool passed_waypoint;
        bool waypoint_position_ready;
        bool rotate_point;
        bool rotate_trigger_ready;
        bool final_waypoint;

        replay_sync_segment_brake();

        replay_transform_keypoint(prev_index, &prev_x, &prev_y, NULL);
        replay_transform_keypoint(cur_index, &target_x, &target_y, NULL);
        final_waypoint = (cur_index + 1u >= g_record_state.keypoint_count);

        segment_dx = target_x - prev_x;
        segment_dy = target_y - prev_y;
        segment_distance = sqrtf(segment_dx * segment_dx +
                                 segment_dy * segment_dy);
        segment_yaw = atan2f(segment_dy, segment_dx);
        upcoming_turn_rad = replay_upcoming_turn_angle(cur_index,
                                                       segment_yaw);

        if (segment_distance < NAV_RECORD_SHORT_SEGMENT_M) {
            if (g_record_keypoints[cur_index].action ==
                NAV_ROUTE_POINT_ACTION_ROTATE720) {
                if (fabsf(input->speed_mps) >
                    NAV_RECORD_ROTATE_TRIGGER_SPEED_MPS) {
                    replay_hold_current_yaw(&out, input);
                    out.region = NAV_REGION_NORMAL;
                    return out;
                }
                return replay_rotate_action_brake_update(input, cur_index);
            }

            replay_fill_waypoint_event(&out, cur_index);
            replay_advance_waypoint();
            if (out.waypoint_action != NAV_ROUTE_POINT_ACTION_NONE) {
                replay_hold_current_yaw(&out, input);
                return out;
            }
            continue;
        }

        target_dx = target_x - input->x_m;
        target_dy = target_y - input->y_m;
        target_distance = sqrtf(target_dx * target_dx +
                                target_dy * target_dy);
        along_remaining = (target_dx * segment_dx +
                           target_dy * segment_dy) / segment_distance;
        rotate_point = (g_record_keypoints[cur_index].action ==
                        NAV_ROUTE_POINT_ACTION_ROTATE720);
        waypoint_reach_radius = reach_radius;
        if (rotate_point) {
            waypoint_reach_radius = NAV_RECORD_ROTATE_WAYPOINT_REACHED_M;
            passed_waypoint = waypoint_passed_with_limit(
                prev_x,
                prev_y,
                target_x,
                target_y,
                input->x_m,
                input->y_m,
                NAV_RECORD_ROTATE_PASS_CROSSTRACK_M);
        } else {
            passed_waypoint = waypoint_passed(prev_x,
                                              prev_y,
                                              target_x,
                                              target_y,
                                              input->x_m,
                                              input->y_m);
        }
        waypoint_position_ready =
            (target_distance <= waypoint_reach_radius || passed_waypoint);
        rotate_trigger_ready =
            rotate_point &&
            target_distance <= NAV_RECORD_ROTATE_TRIGGER_DISTANCE_M &&
            fabsf(input->speed_mps) <= NAV_RECORD_ROTATE_TRIGGER_SPEED_MPS;

        if (rotate_point &&
            (target_distance <= NAV_RECORD_ROTATE_CRAWL_DISTANCE_M ||
             passed_waypoint) &&
            !rotate_trigger_ready) {
            if (target_distance <= NAV_RECORD_ROTATE_TRIGGER_DISTANCE_M) {
                out.velocity_cmd = 0.0f;
            } else {
                out.velocity_cmd = (along_remaining >= 0.0f) ?
                    NAV_RECORD_ROTATE_CRAWL_SPEED :
                    -NAV_RECORD_ROTATE_CRAWL_SPEED;
            }
            out.target_yaw_valid = true;
            out.target_yaw_rad = replay_body_yaw_from_path_yaw(segment_yaw);
            out.region = NAV_REGION_NORMAL;
            return out;
        }

        if (final_waypoint &&
            (rotate_point ? rotate_trigger_ready : waypoint_position_ready)) {
            Nav_Output_t brake_out;
            if (rotate_point) {
                return replay_rotate_action_brake_update(input, cur_index);
            }

            replay_advance_waypoint();
            g_replay_final_braking = true;
            g_replay_final_brake_start_time = input->time_ms;
            g_replay_final_brake_yaw =
                replay_body_yaw_from_path_yaw(segment_yaw);
            brake_out = replay_final_brake_update(input);
            replay_fill_waypoint_event(&brake_out, cur_index);
            return brake_out;
        }

        if (!final_waypoint &&
            (rotate_point ? rotate_trigger_ready : waypoint_position_ready)) {
            if (rotate_point) {
                return replay_rotate_action_brake_update(input, cur_index);
            }

            replay_fill_waypoint_event(&out, cur_index);
            replay_advance_waypoint();
            replay_hold_current_yaw(&out, input);
            return out;
        }

        cross_track_error =
            -sinf(segment_yaw) * (input->x_m - prev_x) +
             cosf(segment_yaw) * (input->y_m - prev_y);
        yaw_correction = clamp(-NAV_RECORD_CROSSTRACK_GAIN * cross_track_error,
                               -NAV_RECORD_CROSSTRACK_LIMIT_RAD,
                               NAV_RECORD_CROSSTRACK_LIMIT_RAD);
        target_yaw_cmd =
            replay_body_yaw_from_path_yaw(segment_yaw + yaw_correction);
        yaw_error = nav_wrap_pi(target_yaw_cmd - input->yaw_rad);
        abs_yaw_error = fabsf(yaw_error);

        if (abs_yaw_error >= NAV_RECORD_TURN_IN_PLACE_YAW_RAD) {
            out.velocity_cmd = NAV_RECORD_TURN_SPEED;
        } else {
            out.velocity_cmd = NAV_RECORD_STRAIGHT_SPEED;
            if (abs_yaw_error >= NAV_RECORD_SLOW_YAW_ERROR_RAD) {
                out.velocity_cmd = NAV_RECORD_CORNER_SPEED;
            }
        }
        braking_before_turn =
            replay_apply_turn_prebrake(&out,
                                       input,
                                       segment_distance,
                                       target_distance,
                                       upcoming_turn_rad);
        if (braking_before_turn) {
            target_yaw_cmd = input->yaw_rad;
        }
        braking_before_rotate =
            replay_apply_rotate_prebrake(&out,
                                         input,
                                         segment_distance,
                                         cur_index,
                                         target_distance);
        if (braking_before_rotate) {
            target_yaw_cmd = input->yaw_rad;
        }
        out.target_yaw_valid = true;
        out.target_yaw_rad = target_yaw_cmd;
        out.region = NAV_REGION_NORMAL;
        return out;
    }

    g_record_state.mode = NAV_ROUTE_READY;
    g_record_state.replay_index = 0u;
    replay_reset_final_brake();
    replay_reset_segment_brake();
    out.finished = true;
    replay_hold_current_yaw(&out, input);
    return out;
}

void nav_route_replay_stop(void)
{
    nav_stop();
    g_record_state.mode = g_record_state.route_ready ?
        NAV_ROUTE_READY : NAV_ROUTE_IDLE;
    g_record_state.replay_index = 0u;
    replay_reset_final_brake();
    replay_reset_segment_brake();
}

Nav_Route_Record_State_t nav_route_record_get_state(void)
{
    return g_record_state;
}
