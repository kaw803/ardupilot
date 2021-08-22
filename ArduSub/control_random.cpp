#include "Sub.h"

/*
 * Init and run calls for guided flight mode
 */

// guided_init - initialise guided controller
bool Sub::random_init(bool ignore_checks)
{
    if (!position_ok() && !ignore_checks) {
        return false;
    }

    // start in position control mode
    guided_pos_control_start();
    gcs().send_text(MAV_SEVERITY_CRITICAL,"hello RANDOM mode");
    return true;
}

void Sub::random_run()
{
    static int   cnt=0;              // 移動時間カウンター
    static float target_yaw = 0.0;   // 最終目標の方向
    static float current_yaw = 0.0;  // 目標に向く前のひとまずの方向
    float yaw = adj360( degrees( ahrs.get_yaw() ) );    // 現在の方向（get_yawの戻り値が-180～+180のため0～360に補正）

    if(cnt==0){
        // 移動時間カウンターが０になったら、次の行き先（向き）を指定する（この例では気まぐれランダム）
        target_yaw = (float)( random() % 360 );         // テキトーに目標を決める（0～359）
        cnt=5000;                                       // 移動時間カウンターを再セット（この設定時間分だけ直進する）
        current_yaw = yaw;
    }

    float cd = abs( adj180( current_yaw - yaw ) );      // 行きたい方向とのズレ（単純な引き算結果だと359-0=359など隣あっていても大きな値になってしまうことがあるため補正）
    float td = adj180( current_yaw - target_yaw );     // 子SUBのペースを考慮して、ゆっくりと方向転換するための制御用
    if( cd < 5.0 && abs( td ) < 5.0 ){
        // 目的の方向を向けているとき（５度以内）
        current_yaw = target_yaw;
        motors.set_forward(0.3);   // 速度アップ
        if(cnt>0) cnt--;              // 移動時間カウンターを進める
    }else{
        // 行きたい方向を向くまで足踏み
        motors.set_forward(0.05);   // 鋭角に曲がるために速度を落とす（速くすると角がラウンドする）
        if(td<0){
            current_yaw = adj360( current_yaw + 0.02 );     // 進行方向に徐々に近づける
        }else{
            current_yaw = adj360( current_yaw - 0.02 );     // 進行方向に徐々に近づける
        }
    }

    motors.set_lateral(0);
    attitude_control.input_euler_angle_roll_pitch_yaw( 0.0f, 0.0f, current_yaw * 1e2f, true);

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "[%04d] TGT(%.1f) CUR(%.1f) YAW(%.1f)", cnt, (double)target_yaw, (double)current_yaw,(double)yaw );
}
