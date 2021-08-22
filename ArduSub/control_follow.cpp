#include "Sub.h"
#include "math.h"


float Sub::adj360(float deg)
{
    if(deg<0.0){
        deg+=360.0;
    }else if(deg>360.0){
        deg-=360.0;
    }
    return(deg);
}

float Sub::adj180(float deg)
{
    if(deg<-180.0){
        deg+=360.0;
    }else if(deg>180.0){
        deg-=360.0;
    }
    return(deg);
}

float Sub::deg2rad(float deg){
    return( adj360( deg ) * 3.141592653589793f / 180.0f );
}

bool Sub::follow_recieve_data( float master_north, float master_east, float master_down, float follow_north, float follow_east, float follow_down )
{
    FM_master_north = master_north;    FM_master_east  = master_east;    FM_master_down  = master_down;
    FM_follow_north = follow_north;    FM_follow_east  = follow_east;    FM_follow_down  = follow_down;

    float dN = FM_master_north - FM_follow_north;
    float dE = FM_master_east  - FM_follow_east;

    FM_yaw = adj360( degrees( atan2f( dE, dN ) ) );      // 子から見た親の位置(deg)
    FM_distance = sqrtf((dN*dN) + (dE*dE));

    // 親SUBの位置座標の変化量から進行方向を推定する
    dN = FM_master_north - FM_last_master_north;
    dE = FM_master_east  - FM_last_master_east;
    float d = sqrtf((dN*dN) + (dE*dE));
    if( d > 0.01 ){
        // ある程度動いているときのみ更新する（親SUBの方向がダイレクトに分かった方が好都合）
        FM_master_heading = adj360( degrees( atan2f( dE, dN ) ) );  // 親SUBが向いている方向
    }

    // 親SUBの今回の位置を保存しておく
    FM_last_master_north = FM_master_north;
    FM_last_master_east  = FM_master_east;

    gcs().send_text(MAV_SEVERITY_CRITICAL, "%0.1f, %0.1f (%0.1f)", dN, dE, FM_yaw );

    return true;
}


/*
bool Sub::follow_recieve_data( float master_north, float master_east, float master_down, float follow_north, float follow_east, float follow_down )
{
    FM_master_north = master_north;    FM_master_east  = master_east;    FM_master_down  = master_down;
    FM_follow_north = follow_north;    FM_follow_east  = follow_east;    FM_follow_down  = follow_down;

    float dN = FM_master_north - FM_follow_north;
    float dE = FM_master_east  - FM_follow_east;

    // FM_yaw = adj360( atan2f( dE, dN ) * 180.0f / 3.141592653589793f );      // 子から見た親の位置(deg)
    FM_yaw = adj360( degrees( atan2f( dE, dN ) ) );      // 子から見た親の位置(deg)
    FM_distance = sqrtf((dN*dN) + (dE*dE));

    // 親SUBの位置座標の変化量から進行方向を推定する
    dN = FM_master_north - FM_last_master_north;
    dE = FM_master_east  - FM_last_master_east;
    FM_master_heading = adj360( degrees( atan2f( dE, dN ) ) );      // 子から見た親の位置

    // 親SUBの今回の位置を保存しておく
    FM_last_master_north = FM_master_north;
    FM_last_master_east  = FM_master_east;

    gcs().send_text(MAV_SEVERITY_CRITICAL, "%0.1f, %0.1f (%0.1f)", dN, dE, FM_yaw );

    return true;
}
*/


/*
 * Init and run calls for follow flight mode
 */

// follow_init - initialise follow controller
bool Sub::follow_init()
{
    if (!position_ok()) {
        return false;
    }
    guided_pos_control_start();
    gcs().send_text(MAV_SEVERITY_CRITICAL,"hello FOLLOW mode");
    return true;
}

// follow_run - runs the follow controller
// should be called at 100hz or more

void Sub::follow_run()
{
    motors.set_forward(1);
    motors.set_lateral(0);
    
    attitude_control.input_euler_angle_roll_pitch_yaw( 0.0f, 0.0f, FM_yaw * 1e2f, true);
}