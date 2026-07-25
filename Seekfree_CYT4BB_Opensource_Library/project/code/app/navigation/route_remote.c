#include "route_remote.h"

#include "../../hmi/indicator/led_buzzer.h"
#include "../remote/remote_comm.h"

/*
 * key[2]: add record point;
 * key[3]: cycle next record point action:
 *         NONE -> ROTATE720 -> SLOW_DOWN -> STEP_START -> STEP_END
 *         -> BUMPY_START -> BUMPY_END -> NONE.
 * switch_key[2]: replay;
 * switch_key[3]: record mode.
 */
#define ROUTE_RECORD_KEY        2u
#define ROUTE_ACTION_KEY        3u
#define ROUTE_PLAY_SWITCH       2u
#define ROUTE_RECORD_SWITCH     3u

static uint8 g_prev_key[4] = {0u, 0u, 0u, 0u};
static uint8 g_prev_record_switch;
static uint8 g_prev_play_switch;
static bool g_prev_connected;
static Nav_Route_Point_Action_t g_next_keypoint_action =
    NAV_ROUTE_POINT_ACTION_NONE;
static bool g_remote_started_replay;

static Nav_Route_Point_Action_t route_next_action(
    Nav_Route_Point_Action_t action)
{
    switch (action) {
    case NAV_ROUTE_POINT_ACTION_NONE:
        return NAV_ROUTE_POINT_ACTION_ROTATE720;
    case NAV_ROUTE_POINT_ACTION_ROTATE720:
        return NAV_ROUTE_POINT_ACTION_SLOW_DOWN;
    case NAV_ROUTE_POINT_ACTION_SLOW_DOWN:
        return NAV_ROUTE_POINT_ACTION_STEP_START;
    case NAV_ROUTE_POINT_ACTION_STEP_START:
        return NAV_ROUTE_POINT_ACTION_STEP_END;
    case NAV_ROUTE_POINT_ACTION_STEP_END:
        return NAV_ROUTE_POINT_ACTION_BUMPY_START;
    case NAV_ROUTE_POINT_ACTION_BUMPY_START:
        return NAV_ROUTE_POINT_ACTION_BUMPY_END;
    case NAV_ROUTE_POINT_ACTION_BUMPY_END:
    default:
        return NAV_ROUTE_POINT_ACTION_NONE;
    }
}

static Beep_Pattern_t route_action_beep(Nav_Route_Point_Action_t action)
{
    switch (action) {
    case NAV_ROUTE_POINT_ACTION_NONE:
        return BEEP_SHORT;
    case NAV_ROUTE_POINT_ACTION_ROTATE720:
        return BEEP_DOUBLE;
    case NAV_ROUTE_POINT_ACTION_SLOW_DOWN:
        return BEEP_TRIPLE;
    case NAV_ROUTE_POINT_ACTION_STEP_START:
        return BEEP_TRIPLE;
    case NAV_ROUTE_POINT_ACTION_STEP_END:
        return BEEP_LONG;
    case NAV_ROUTE_POINT_ACTION_BUMPY_START:
        return BEEP_DOUBLE_LONG;
    case NAV_ROUTE_POINT_ACTION_BUMPY_END:
        return BEEP_LONG;
    default:
        return BEEP_ERROR;
    }
}

static bool remote_key_rising(const Remote_State_t *remote, uint8 index)
{
    if (remote == NULL || index >= 4u) {
        return false;
    }

    uint8 now = remote->key[index] != 0u ? 1u : 0u;
    bool rising = (now != 0u && g_prev_key[index] == 0u);
    g_prev_key[index] = now;

    return rising;
}

static void remote_key_sync(const Remote_State_t *remote, uint8 index)
{
    if (remote == NULL || index >= 4u) {
        return;
    }

    g_prev_key[index] = remote->key[index] != 0u ? 1u : 0u;
}

void route_remote_update(const Nav_Input_t *input)
{
    const Remote_State_t *remote = remote_comm_get_state();
    uint8 record_switch;
    uint8 play_switch;
    bool record_switch_rising;
    bool record_switch_falling;
    bool play_switch_rising;
    Nav_Route_Record_State_t state;

    if (remote == NULL) {
        return;
    }

    record_switch = remote->switch_key[ROUTE_RECORD_SWITCH] != 0u ? 1u : 0u;
    play_switch = remote->switch_key[ROUTE_PLAY_SWITCH] != 0u ? 1u : 0u;

    if (!remote->connected) {
        g_next_keypoint_action = NAV_ROUTE_POINT_ACTION_NONE;
        g_prev_connected = false;
        state = nav_route_record_get_state();
        if (g_remote_started_replay && state.mode == NAV_ROUTE_REPLAYING) {
            nav_route_replay_stop();
        }
        g_remote_started_replay = false;
        return;
    }

    if (input == NULL) {
        return;
    }

    if (!g_prev_connected) {
        g_prev_connected = true;
        g_prev_record_switch = record_switch;
        g_prev_play_switch = play_switch;
        remote_key_sync(remote, ROUTE_RECORD_KEY);
        remote_key_sync(remote, ROUTE_ACTION_KEY);
        return;
    }

    record_switch_rising = (record_switch != 0u && g_prev_record_switch == 0u);
    record_switch_falling = (record_switch == 0u && g_prev_record_switch != 0u);
    play_switch_rising = (play_switch != 0u && g_prev_play_switch == 0u);
    g_prev_record_switch = record_switch;
    g_prev_play_switch = play_switch;

    state = nav_route_record_get_state();
    if (g_remote_started_replay && state.mode != NAV_ROUTE_REPLAYING) {
        g_remote_started_replay = false;
    }

    if (record_switch_rising) {
        g_next_keypoint_action = NAV_ROUTE_POINT_ACTION_NONE;
        g_remote_started_replay = false;
        nav_route_replay_stop();
        if (nav_route_record_start(input)) {
            buzzer_beep(BEEP_TRIPLE);
        }
        remote_key_sync(remote, ROUTE_RECORD_KEY);
        remote_key_sync(remote, ROUTE_ACTION_KEY);
    }

    if (record_switch != 0u) {
        state = nav_route_record_get_state();
        if (state.mode != NAV_ROUTE_RECORDING) {
            g_next_keypoint_action = NAV_ROUTE_POINT_ACTION_NONE;
            if (nav_route_record_start(input)) {
                buzzer_beep(BEEP_TRIPLE);
            }
            remote_key_sync(remote, ROUTE_RECORD_KEY);
            remote_key_sync(remote, ROUTE_ACTION_KEY);
        }

        if (state.mode == NAV_ROUTE_RECORDING) {
            if (remote_key_rising(remote, ROUTE_ACTION_KEY)) {
                g_next_keypoint_action =
                    route_next_action(g_next_keypoint_action);
                buzzer_beep(route_action_beep(g_next_keypoint_action));
            }

            if (remote_key_rising(remote, ROUTE_RECORD_KEY)) {
                Nav_Route_Point_Action_t action = g_next_keypoint_action;

                if (!nav_route_record_keypoint_with_action(input, action)) {
                    buzzer_beep(BEEP_ERROR);
                } else {
                    buzzer_beep(route_action_beep(action));
                    g_next_keypoint_action = NAV_ROUTE_POINT_ACTION_NONE;
                }
            }
        }
        return;
    }

    if (record_switch_falling) {
        g_next_keypoint_action = NAV_ROUTE_POINT_ACTION_NONE;
        state = nav_route_record_get_state();
        if (state.mode == NAV_ROUTE_RECORDING) {
            if (nav_route_record_finish()) {
                buzzer_beep(BEEP_LONG);
            } else {
                buzzer_beep(BEEP_ERROR);
            }
        }
    }

    state = nav_route_record_get_state();
    if (g_remote_started_replay &&
        play_switch == 0u &&
        state.mode == NAV_ROUTE_REPLAYING) {
        nav_route_replay_stop();
        g_remote_started_replay = false;
        return;
    }

    if (play_switch_rising && state.mode != NAV_ROUTE_RECORDING) {
        g_next_keypoint_action = NAV_ROUTE_POINT_ACTION_NONE;
        if (nav_route_replay_start(input)) {
            g_remote_started_replay = true;
            buzzer_beep(BEEP_DOUBLE_LONG);
        }
    }
}
