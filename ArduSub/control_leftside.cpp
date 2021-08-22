#include "Sub.h"

/*
 * Init and run calls for guided flight mode
 */

// guided_init - initialise guided controller
bool Sub::leftside_init()
{
    if (!position_ok()) {
        return false;
    }

    // start in position control mode
    guided_pos_control_start();
    gcs().send_text(MAV_SEVERITY_CRITICAL,"hello LEFTSIDE mode");
    return true;
}

void Sub::leftside_run()
{
    float follow_yaw = 40.0f;           // 親SUBとの位置関係 -90～0[deg]:右後ろ、0～90[deg]：左後ろ（0：真後ろ、180：真ん前）
    float follow_distance = 10.0f;      // 親SUBとの距離[m]

    // 目指すポイント
    float target_north = FM_master_north - follow_distance * cosf( deg2rad(follow_yaw + FM_master_heading) );
    float target_east  = FM_master_east  - follow_distance * sinf( deg2rad(follow_yaw + FM_master_heading) );
    float dN = target_north - FM_follow_north;
    float dE = target_east  - FM_follow_east;
    float yaw = atan2f( dE, dN ) * 180.0f / 3.141592653589793f;         // ターゲット方向(deg)

    // 機首は親SUBを真似る
    attitude_control.input_euler_angle_roll_pitch_yaw( 0.0f, 0.0f, FM_master_heading * 1e2f, true);

    float target_yaw = deg2rad( yaw - FM_master_heading );              // 機首は親SUBを真似ながら、ターゲット方向に移動する
    float forward_power = cosf(target_yaw);
    float lateral_power = sinf(target_yaw);

    motors.set_forward(forward_power);
    motors.set_lateral(lateral_power);
}
