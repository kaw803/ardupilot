#include "Sub.h"

/*
 * Init and run calls for guided flight mode
 */

// guided_init - initialise guided controller
bool Sub::heading_init(bool ignore_checks)
{
    if (!position_ok() && !ignore_checks) {
        return false;
    }

    // start in position control mode
    guided_pos_control_start();
    gcs().send_text(MAV_SEVERITY_CRITICAL,"hello HEADING mode");
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Sub::pentagon_run()
{
    /// Pentagon
    static int   cnt = 3000;             // 移動時間カウンター
    static float target_yaw = 0;  // 現在進んでいる方向
    static int   pentagon_cnt = 0;
    if(cnt==0){
        // 移動時間カウンターが0になったら、次の行き先（向き）を指定する
        if(pentagon_cnt == 0) {
          target_yaw = -72;  /// 時間カウントが終わったら0度にする
        }else if(pentagon_cnt == 1){
          target_yaw = -144;  /// 時間カウントが終わったら-108度にする
        }else if (pentagon_cnt == 2){
          target_yaw = -216;  /// 時間カウントが終わったら-216度にする
        }else if (pentagon_cnt == 3){
          target_yaw = -288;  /// 時間カウントが終わったら-324度にする
        }else if (pentagon_cnt == 4){
          target_yaw = 0;  /// 時間カウントが終わったら-324度にする
        }
        cnt=10000;              // 移動時間カウンターを再セット（この設定時間分だけ直進する）
        pentagon_cnt++;
        if(pentagon_cnt>4){
            pentagon_cnt=0;
        }  
    }
    motors.set_forward(0.3);   // 速度アップ
    if(cnt) cnt--;              // 移動時間カウンターを進める
    motors.set_lateral(0);
    attitude_control.input_euler_angle_roll_pitch_yaw( 0.0f, 0.0f, target_yaw * 1e2f, true);
    gcs().send_text(MAV_SEVERITY_INFO, "[%04d] TGT(%.1f)", cnt, (double)target_yaw);
}

void Sub::heading_run()
{
    static int   cnt=0;             // 移動時間カウンター
    static float target_yaw = 0.0;  // 現在進んでいる方向

    Vector3f ahrs_pos_ned;
    float x, y, dist;
    static float last_dist, dx=0.0;
    ahrs.get_relative_position_NED_origin(ahrs_pos_ned);
    x = ahrs_pos_ned[0];
    y = ahrs_pos_ned[1];
    dist = sqrtf( x*x + y*y );
    dx = dist - last_dist;      // dx>0:遠ざかっている、dx<0:近づいている
    gcs().send_text(MAV_SEVERITY_CRITICAL, "%4d: [%3.1f] (%.3f)", cnt,(double)dist, (double)dx );
    if( dist < 1.0){
        cnt = 5000;
    }
    last_dist = dist;

    if(cnt==0){
        // 移動時間カウンターが０になったら、次の行き先（向き）を指定する（この例では気まぐれランダム）
        target_yaw = (float)( random() % 360 );         // テキトーに目標を決める（0～359）
        cnt=1000;                                       // 移動時間カウンターを再セット（この設定時間分だけ直進する）
    }

    //float yaw = adj360( degrees( ahrs.get_yaw() ) );    // 現在の方向（get_yawの戻り値が-180～+180のため0～360に補正）
    //float d   = abs( adj180( target_yaw - yaw ) );      // 行きたい方向とのズレ（単純な引き算結果だと359-0=359など隣あっていても大きな値になってしまうことがあるため補正）

    if( cnt > 1000 ){
        motors.set_forward(1);   // 初期位置まで全速力
    }else if( dx < 0 ){
        motors.set_forward(1);   // 原点に近づいているときは頑張る
    }else{
        motors.set_forward(0.1);    // 遠ざかっているときは嫌がる（遅くなる）
    }
    if(cnt) cnt--;              // 移動時間カウンターを進める
/*
    if( d < 5.0 ){
        // 目的の方向を向けているとき（５度以内）
        if( dx < 0 ){
            motors.set_forward(1);   // 原点に近づいているときは頑張る
        }else{
            motors.set_forward(0.5);    // 遠ざかっているときは嫌がる（遅くなる）
        }
        if(cnt) cnt--;              // 移動時間カウンターを進める
    }else{
        // 行きたい方向を向くまで足踏み
        motors.set_forward(0.01);   // 鋭角に曲がるために速度を落とす（速くすると角がラウンドする）
        // if(cnt) cnt--;           // 方向転換中は移動距離に含めないよう、あえてカウンター更新しない
    }
*/
    motors.set_lateral(0);
    attitude_control.input_euler_angle_roll_pitch_yaw( 0.0f, 0.0f, target_yaw * 1e2f, true);
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "[%04d] TGT(%.1f) YAW(%.1f) d(%.1f)", cnt, (double)target_yaw, (double)yaw, (double)d );
}

void Sub::heading_run2()
{
    static int cnt=0;
    static float target_yaw = 0.0;

//    float target_yaw;
    float target_pitch;
    float target_roll;
    target_roll = 0; //(3.1415/4);
    target_pitch = 0; //(3.1415/4);
//    target_yaw = 3.1415*3/2;  
    target_roll = degrees(target_roll);
    target_pitch = degrees(target_pitch);
    //    target_yaw = degrees(target_yaw);

    float yaw = degrees(ahrs.get_yaw());
    motors.set_lateral(0);

    if( cnt>6000 ){
        cnt = 0;
        if( abs(yaw-(target_yaw-180))<5 ){
            cnt=0;
        }
    }else if( cnt==2000 ){
        motors.set_forward(0);
        target_yaw = (float)(((int)target_yaw + 45)%360);
        if(target_yaw>180.0)    target_yaw-=360;
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll * 1e2f, target_pitch * 1e2f, (target_yaw-0) * 1e2f, true);
    }else if( cnt<2000 ){
        motors.set_forward(1000);
    }else{
    }

/*
    if(cnt>5000){
        cnt=-5000;
    }else if( cnt>0 ){
        motors.set_forward(1000);
    }else{
        motors.set_forward(-1000);
    }
*/
    //gcs().send_text(MAV_SEVERITY_CRITICAL,"HEDING mode is running! %d",(int)cnt);
    cnt++;
    
    Vector3f ahrs_pos_ned;
    float x,y,z;
    ahrs.get_relative_position_NED_origin(ahrs_pos_ned);
    x = ahrs_pos_ned[0];
    y = ahrs_pos_ned[1];
    z = ahrs_pos_ned[2];
    gcs().send_text(MAV_SEVERITY_CRITICAL, "%d: %3.1f,%3.1f,%3.1f %d", cnt,(double)x,(double)y,(double)z, (int)target_yaw );
    //gcs().send_text(MAV_SEVERITY_CRITICAL,"cnt:%d  yaw:%d  target:%d  (%d)",cnt,(int)yaw,(int)target_yaw-180,(int)(yaw-(target_yaw-180)));
}
