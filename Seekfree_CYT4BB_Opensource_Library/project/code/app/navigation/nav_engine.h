#ifndef APP_NAVIGATION_NAV_ENGINE_H
#define APP_NAVIGATION_NAV_ENGINE_H

#include "zf_common_typedef.h"

#include "../../common/types.h"

typedef enum {
    NAV_ACTION_IDLE = 0,
    NAV_ACTION_GO_STRAIGHT,
    NAV_ACTION_TURN,
    NAV_ACTION_WAIT_LANDMARK,
    NAV_ACTION_STOP
} Nav_Action_t;

typedef enum {
    NAV_REGION_NONE = 0,
    NAV_REGION_NORMAL,
    NAV_REGION_ROTATE,
    NAV_REGION_JUMP,
    NAV_REGION_SPEED_BUMP,
    NAV_REGION_SINGLE_BRIDGE,
    NAV_REGION_UPHILL,
    NAV_REGION_GRASS
} Nav_Region_t;

typedef enum {
    NAV_LANDMARK_NONE = 0,
    NAV_LANDMARK_CONE,
    NAV_LANDMARK_WHITE_CIRCLE,
    NAV_LANDMARK_STEP
} Nav_Landmark_t;

typedef enum {
    NAV_ROUTE_POINT_ACTION_NONE = 0,
    NAV_ROUTE_POINT_ACTION_ROTATE720,
    NAV_ROUTE_POINT_ACTION_STEP_START,
    NAV_ROUTE_POINT_ACTION_STEP_END,
    NAV_ROUTE_POINT_ACTION_BUMPY_START,
    NAV_ROUTE_POINT_ACTION_BUMPY_END,
    NAV_ROUTE_POINT_ACTION_SLOW_DOWN
} Nav_Route_Point_Action_t;

typedef struct {
    Nav_Action_t action;
    float target_distance_m;
    float target_yaw_deg;
    float target_speed;
    uint32 timeout_ms;
    Nav_Landmark_t landmark;
    Nav_Region_t region;
} Nav_Segment_t;

typedef struct {
    float x_m;
    float y_m;
    float distance_m;
    float speed_mps;
    float yaw_rad;
    uint32 time_ms;
    Nav_Landmark_t landmark;
    bool obstacle_close;
} Nav_Input_t;

typedef struct {
    float yaw_kp;
    float turn_kp;
    float steering_limit;
    float distance_tolerance_m;
    float yaw_tolerance_rad;
} Nav_Config_t;

typedef struct {
    bool active;
    bool finished;
    bool safety_stop;
    uint8 segment_index;
    Nav_Action_t action;
    Nav_Region_t region;
    Nav_Region_t previous_region;
    /* Set for the nav_update/nav_start cycle that changes region. */
    bool region_entered;
    bool region_exited;
    float segment_distance_m;
    float region_distance_m;
    uint32 region_elapsed_ms;
    float yaw_error_rad;
} Nav_State_t;

typedef struct {
    float velocity_cmd;
    float steering_cmd;
    bool target_yaw_valid;
    float target_yaw_rad;
    bool finished;
    bool safety_stop;
    Nav_Region_t region;
    bool region_entered;
    bool region_exited;
    bool waypoint_entered;
    uint8 waypoint_index;
    Nav_Route_Point_Action_t waypoint_action;
} Nav_Output_t;

void         nav_init                   (const Nav_Config_t *config);
void         nav_set_route              (const Nav_Segment_t *route, uint8 route_len);
void         nav_start                  (const Nav_Input_t *input);
void         nav_stop                   (void);
Nav_Output_t nav_update                 (const Nav_Input_t *input);
Nav_State_t  nav_get_state              (void);
Nav_Config_t  nav_get_config            (void);
void          nav_set_config            (const Nav_Config_t *config);
Nav_Config_t *nav_config                (void);

#endif
