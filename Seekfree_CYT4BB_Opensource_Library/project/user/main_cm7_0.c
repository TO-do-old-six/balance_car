#include "zf_common_headfile.h"
#include "../code/sensors/imu/imu.h"
#include "../code/common/types.h"
#include "../code/app/navigation/nav_engine.h"
#include "../code/app/navigation/nav_adapter.h"
#include "../code/app/navigation/nav_route_record.h"
#include "../code/app/navigation/route_remote.h"
#include "../code/app/vision/vision_pipeline.h"
#include "../code/app/robot_control/robot_control.h"
#include "../code/app/robot_control/small_driver_uart_control.h"
#include "../code/app/remote/remote_comm.h"
#include "../code/app/remote/remote_debug.h"
#include "../code/app/robot_control/jump.h"
#include "../code/app/robot_control/track_elements.h"
#include "../code/control/leg/angle_offset.h"
#include "../code/hmi/indicator/led_buzzer.h"
#include "../code/hmi/input/input_handler.h"
#include <math.h>

/* 手动测试开关: 标定完成后直接激活, 不依赖导航 */
#define BRIDGE_MANUAL_TEST         0
#define SINGLE_BRIDGE_MANUAL_TEST  0
#define BUMPY_MANUAL_TEST          0
#define BOARD_REPLAY_KEY KEY_1
#define BOARD_ROUTE_SAVE_KEY_2 KEY_2
#define BOARD_ROUTE_SAVE_KEY_3 KEY_3
#define BOARD_ROUTE_SAVE_KEY_4 KEY_4
#define BOARD_ROUTE_SAVE_SLOT_2 0u
#define BOARD_ROUTE_SAVE_SLOT_3 1u
#define BOARD_ROUTE_SAVE_SLOT_4 2u
#define BOARD_ROUTE_SAVE_SLOT_COUNT 3u
#define BOARD_KEY_SCAN_PERIOD_MS 1u

#define CM7_0_READY_MAGIC 0x43373031u

#pragma location = 0x28006C00
__no_init volatile uint32 g_cm7_0_ready;

static void cm7_0_set_ready(uint32 value)
{
    g_cm7_0_ready = value;
    SCB_CleanDCache_by_Addr((uint32 *)&g_cm7_0_ready, 32u);
}

Ctrl_Input_t   g_ctrl;
static Nav_Input_t    g_nav_input;
static Vision_Result_t g_vision;

Motor_cmd_duty_t g_motor_cmd;

Sensor_data_t g_sensor_data;

Move_cmd_t g_move_cmd;

static uint8 g_board_replay_slot = BOARD_ROUTE_SAVE_SLOT_2;

#define COURSE_STEP_SPEED             (-0.15f)
#define COURSE_STEP_COUNT             3u
#define COURSE_STEP_TIMEOUT_MS        10000u
#define COURSE_BUMPY_RUN_MS           4500u
#define COURSE_BUMPY_TIMEOUT_MS       12000u
#define COURSE_STABLE_TICKS           200u
#define COURSE_STABLE_PITCH_RAD       (10.0f * DEG_TO_RAD)
#define COURSE_STABLE_ROLL_RAD        (10.0f * DEG_TO_RAD)
#define COURSE_STABLE_GYRO_RAD_S      0.8f
#define COURSE_RECOVER_MS             500u

typedef enum {
    COURSE_ELEMENT_NAV = 0,
    COURSE_ELEMENT_STEP,
    COURSE_ELEMENT_BUMPY,
    COURSE_ELEMENT_RECOVER
} Course_Element_State_t;

static Course_Element_State_t g_course_element_state = COURSE_ELEMENT_NAV;
static uint32 g_course_element_start_ms = 0u;
static uint16 g_course_element_stable_ticks = 0u;

static void course_element_hold_heading(Ctrl_Input_t *ctrl,
                                        const Nav_Input_t *input)
{
    if (ctrl == NULL || input == NULL) {
        return;
    }

    ctrl->velocity_cmd = 0.0f;
    ctrl->steering_cmd = 0.0f;
    ctrl->yaw_target_valid = true;
    ctrl->yaw_target_rad = input->yaw_rad;
}

static bool course_element_stable_update(const Ctrl_Input_t *ctrl)
{
    bool stable;

    if (ctrl == NULL) {
        g_course_element_stable_ticks = 0u;
        return false;
    }

    stable = fabsf(ctrl->body_pitch) <= COURSE_STABLE_PITCH_RAD &&
             fabsf(ctrl->body_roll) <= COURSE_STABLE_ROLL_RAD &&
             fabsf(ctrl->gyro_pitch_rate) <= COURSE_STABLE_GYRO_RAD_S &&
             fabsf(ctrl->gyro_roll_rate) <= COURSE_STABLE_GYRO_RAD_S;

    if (stable) {
        if (g_course_element_stable_ticks < COURSE_STABLE_TICKS) {
            g_course_element_stable_ticks++;
        }
    } else {
        g_course_element_stable_ticks = 0u;
    }

    return g_course_element_stable_ticks >= COURSE_STABLE_TICKS;
}

static uint32 course_element_elapsed_ms(const Nav_Input_t *input)
{
    if (input == NULL) {
        return 0u;
    }

    return (uint32)(input->time_ms - g_course_element_start_ms);
}

static void course_element_begin(Course_Element_State_t state,
                                 const Nav_Input_t *input)
{
    nav_adapter_set_odom_hold(true);
    g_course_element_state = state;
    g_course_element_start_ms = input != NULL ? input->time_ms : 0u;
    g_course_element_stable_ticks = 0u;
}

static void course_element_begin_step(const Nav_Input_t *input)
{
    course_element_begin(COURSE_ELEMENT_STEP, input);
    jump_start(COURSE_STEP_SPEED, COURSE_STEP_COUNT);
}

static void course_element_begin_bumpy(const Nav_Input_t *input)
{
    course_element_begin(COURSE_ELEMENT_BUMPY, input);
    track_bumpy_activate();
    track_bumpy_apply_compliance();
}

static bool course_element_finish_at_action(
    const Nav_Input_t *input,
    Nav_Route_Point_Action_t exit_action)
{
    bool anchored = nav_route_replay_anchor_to_next_action(input, exit_action);

    nav_adapter_reset_odom_base();
    nav_adapter_set_odom_hold(false);
    g_course_element_state = COURSE_ELEMENT_NAV;
    g_course_element_stable_ticks = 0u;

    if (!anchored) {
        nav_route_replay_stop();
        buzzer_beep(BEEP_ERROR);
        return false;
    }

    buzzer_beep(BEEP_SHORT);
    return true;
}

static void course_element_enter_recover(const Nav_Input_t *input)
{
    jump_stop();
    track_bumpy_restore_stiffness();
    track_bumpy_deactivate();
    nav_adapter_reset_odom_base();
    nav_adapter_set_odom_hold(false);
    nav_route_replay_stop();
    g_course_element_state = COURSE_ELEMENT_RECOVER;
    g_course_element_start_ms = input != NULL ? input->time_ms : 0u;
    g_course_element_stable_ticks = 0u;
    buzzer_beep(BEEP_ERROR);
}

static void course_handle_region_event(const Nav_Output_t *nav_out)
{
    if (nav_out == NULL) {
        return;
    }

    if (nav_out->region_entered) {
        switch (nav_out->region) {
        case NAV_REGION_ROTATE:
            track_rotate720_start();
            break;
        case NAV_REGION_SINGLE_BRIDGE:
        case NAV_REGION_UPHILL:
            track_bridge_climb_activate();
            break;
        default:
            break;
        }
    }

    if (nav_out->region_exited) {
        switch (nav_out->region) {
        case NAV_REGION_SINGLE_BRIDGE:
        case NAV_REGION_UPHILL:
            track_bridge_climb_deactivate();
            break;
        case NAV_REGION_ROTATE:
            track_rotate720_reset();
            break;
        default:
            break;
        }
    }
}

static void course_handle_waypoint_action(const Nav_Output_t *nav_out,
                                          const Nav_Input_t *input)
{
    if (nav_out == NULL || input == NULL || !nav_out->waypoint_entered) {
        return;
    }

    switch (nav_out->waypoint_action) {
    case NAV_ROUTE_POINT_ACTION_ROTATE720:
        track_rotate720_reset();
        if (nav_out->target_yaw_valid) {
            track_rotate720_start_with_target(nav_out->target_yaw_rad);
        } else {
            track_rotate720_start();
        }
        break;
    case NAV_ROUTE_POINT_ACTION_STEP_START:
        course_element_begin_step(input);
        break;
    case NAV_ROUTE_POINT_ACTION_BUMPY_START:
        course_element_begin_bumpy(input);
        break;
    default:
        break;
    }
}

static void course_element_update(Ctrl_Input_t *ctrl, Nav_Input_t *input)
{
    Nav_Route_Record_State_t route_state = nav_route_record_get_state();

    if (ctrl == NULL || input == NULL) {
        return;
    }

    if (route_state.mode != NAV_ROUTE_REPLAYING &&
        g_course_element_state != COURSE_ELEMENT_NAV &&
        g_course_element_state != COURSE_ELEMENT_RECOVER) {
        jump_stop();
        track_bumpy_restore_stiffness();
        track_bumpy_deactivate();
        nav_adapter_reset_odom_base();
        nav_adapter_set_odom_hold(false);
        g_course_element_state = COURSE_ELEMENT_NAV;
        g_course_element_stable_ticks = 0u;
    }

    switch (g_course_element_state) {
    case COURSE_ELEMENT_NAV:
        if (route_state.mode == NAV_ROUTE_REPLAYING) {
            Nav_Output_t nav_out = nav_route_replay_update(input);

            nav_apply_ctrl(ctrl, &nav_out);
            course_handle_region_event(&nav_out);
            course_handle_waypoint_action(&nav_out, input);
        }
        break;

    case COURSE_ELEMENT_STEP:
        course_element_hold_heading(ctrl, input);
        if (!jump_is_active() &&
            course_element_stable_update(ctrl)) {
            (void)course_element_finish_at_action(
                input,
                NAV_ROUTE_POINT_ACTION_STEP_END);
        } else if (course_element_elapsed_ms(input) >=
                   COURSE_STEP_TIMEOUT_MS) {
            course_element_enter_recover(input);
        }
        break;

    case COURSE_ELEMENT_BUMPY:
        course_element_hold_heading(ctrl, input);
        if ((course_element_elapsed_ms(input) >= COURSE_BUMPY_RUN_MS ||
             !track_bumpy_is_active()) &&
            course_element_stable_update(ctrl)) {
            track_bumpy_restore_stiffness();
            track_bumpy_deactivate();
            (void)course_element_finish_at_action(
                input,
                NAV_ROUTE_POINT_ACTION_BUMPY_END);
        } else if (course_element_elapsed_ms(input) >=
                   COURSE_BUMPY_TIMEOUT_MS) {
            course_element_enter_recover(input);
        }
        break;

    case COURSE_ELEMENT_RECOVER:
    default:
        course_element_hold_heading(ctrl, input);
        if (course_element_elapsed_ms(input) >= COURSE_RECOVER_MS &&
            course_element_stable_update(ctrl)) {
            g_course_element_state = COURSE_ELEMENT_NAV;
        }
        break;
    }
}

static Beep_Pattern_t board_route_slot_beep(uint8 reserved_slot)
{
    switch (reserved_slot) {
    case BOARD_ROUTE_SAVE_SLOT_2:
        return BEEP_SHORT;
    case BOARD_ROUTE_SAVE_SLOT_3:
        return BEEP_DOUBLE;
    case BOARD_ROUTE_SAVE_SLOT_4:
        return BEEP_TRIPLE;
    default:
        return BEEP_ERROR;
    }
}

static void board_route_select_reserved_slot(uint8 reserved_slot)
{
    if (reserved_slot >= BOARD_ROUTE_SAVE_SLOT_COUNT) {
        buzzer_beep(BEEP_ERROR);
        return;
    }

    g_board_replay_slot = reserved_slot;
    buzzer_beep(board_route_slot_beep(reserved_slot));
}

static void board_replay_key_long_press(void)
{
    Nav_Route_Record_State_t route_state = nav_route_record_get_state();

    if (route_state.mode == NAV_ROUTE_RECORDING) {
        buzzer_beep(BEEP_ERROR);
        return;
    }

    if (!nav_route_record_load_reserved_slot(g_board_replay_slot)) {
        buzzer_beep(BEEP_ERROR);
        return;
    }

    if (nav_route_replay_start(&g_nav_input)) {
        buzzer_beep(BEEP_DOUBLE_LONG);
    } else {
        buzzer_beep(BEEP_ERROR);
    }
}

static void board_route_save_reserved_slot(uint8 reserved_slot)
{
    if (nav_route_record_save_reserved_slot(reserved_slot)) {
        g_board_replay_slot = reserved_slot;
        buzzer_beep(BEEP_LONG);
    } else {
        buzzer_beep(BEEP_ERROR);
    }
}

static void board_route_select_key2_short_press(void)
{
    board_route_select_reserved_slot(BOARD_ROUTE_SAVE_SLOT_2);
}

static void board_route_select_key3_short_press(void)
{
    board_route_select_reserved_slot(BOARD_ROUTE_SAVE_SLOT_3);
}

static void board_route_select_key4_short_press(void)
{
    board_route_select_reserved_slot(BOARD_ROUTE_SAVE_SLOT_4);
}

static void board_route_save_key2_long_press(void)
{
    board_route_save_reserved_slot(BOARD_ROUTE_SAVE_SLOT_2);
}

static void board_route_save_key3_long_press(void)
{
    board_route_save_reserved_slot(BOARD_ROUTE_SAVE_SLOT_3);
}

static void board_route_save_key4_long_press(void)
{
    board_route_save_reserved_slot(BOARD_ROUTE_SAVE_SLOT_4);
}

int main(void)
{
    g_cm7_0_ready = 0u;
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    cm7_0_set_ready(0u);
    zf_log(0, "CM7_1 debugger-managed.");
    remote_debug_init();

    if(!imu_init())
    {
        zf_log(0, "IMU init failed.");
        while(true);
    }

    nav_init(NULL);
    vision_init();
    vision_set_mode(VISION_MODE_MINEFIELD);
    cm7_0_set_ready(CM7_0_READY_MAGIC);

    // UI 运行在 CM7_1，CM7_0 只保留控制/感知/驱动逻辑。
    small_driver_uart_init();
    led_buzzer_init();
    input_handler_init(BOARD_KEY_SCAN_PERIOD_MS);
    input_handler_set_cb(BOARD_REPLAY_KEY, NULL, board_replay_key_long_press);
    input_handler_set_cb(BOARD_ROUTE_SAVE_KEY_2,
                         board_route_select_key2_short_press,
                         board_route_save_key2_long_press);
    input_handler_set_cb(BOARD_ROUTE_SAVE_KEY_3,
                         board_route_select_key3_short_press,
                         board_route_save_key3_long_press);
    input_handler_set_cb(BOARD_ROUTE_SAVE_KEY_4,
                         board_route_select_key4_short_press,
                         board_route_save_key4_long_press);
    remote_comm_init();
    robot_control_init();

    angle_offset_start(NULL);               // 标定全部四个关节 (左前/左后/右前/右后)

    pit_ms_init(PIT_CH1, 1);

    interrupt_global_enable(0);

    while (!angle_offset_is_done()) {
        imu_update(&g_ctrl);
        sensor_cmd_update(&g_ctrl, &g_sensor_data, &g_move_cmd);
        led_buzzer_tick(1u);
        system_delay_ms(1);

        if (angle_offset_has_fault()) {
            zf_log(0, "Angle calibration FAILED (timeout). Halting.");
            while (true);
        }
    }
    zf_log(0, "Angle calibration OK.");

#if BRIDGE_MANUAL_TEST
    //track_bridge_climb_activate();
    //zf_log(0, "Bridge climb manual test started.");
#else
    //track_rotate720_start();   // 测试旋转720
#endif
    //jump_start(-0.27f, 3);//三级台阶
    //jump_start(-2.50f, 1);//直接跳过颠簸

    while(true)
    {

        imu_update(&g_ctrl);// IMU update: for testing, can be moved to timer ISR
        // g_ctrl.body_pitch / body_roll / gyro_pitch_rate / gyro_yaw_rate (rad, rad/s)

        remote_comm_update(&g_ctrl);

        vision_update(&g_vision);

        nav_input_update_from_ctrl(&g_nav_input, &g_ctrl);
        vision_feed_nav_input(&g_nav_input, &g_vision);
        route_remote_update(&g_nav_input);
        input_handler_tick();

        course_element_update(&g_ctrl, &g_nav_input);

        if (track_rotate720_is_active()) {
            g_ctrl.velocity_cmd = 0.0f;
        }
        if (track_rotate720_is_done()) {
            track_rotate720_reset();
        }

        small_driver_get_angle(&small_driver_value);
        sensor_cmd_update(&g_ctrl, &g_sensor_data, &g_move_cmd);

        /* 速度回传, 每 100ms 打印一次 */
        {
            static uint16_t print_cnt = 0;
            if (++print_cnt >= 100) {
                print_cnt = 0;
                printf("speed L=%.2f R=%.2f rad/s\r\n",
                       g_sensor_data.motor_left_speed,
                       g_sensor_data.motor_right_speed);
            }
        }

        led_buzzer_tick(1u);
        system_delay_ms(1);

    }
}
