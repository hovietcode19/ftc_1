package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final String FRONT_LEFT  = "dr1";
    public static final String FRONT_RIGHT = "dr0";
    public static final String BACK_LEFT   = "dr2";
    public static final String BACK_RIGHT  = "dr3";
    public static final String IMU_NAME    = "imu";


    public static final double SLOW_MODE_SCALE = 0.4;
    // bù trượt ngang
    public static final double STRAFE_CORRECTION = 1.1;
    public static final double HEADING_KP = 0.01;
    public static final double HEADING_KD = 0.001;
    public static final double HEADING_KS = 0.03;
    public static final double HEADING_MAX_POWER = 0.5;
    public static final double HEADING_TOLERANCE_DEG = 1.5; // <1.5 degree = True
    public static final double HEADING_RATE_TOLERANCE = 5.0;  // <5.0 = True
    public static final double RATE_FILTER = 0.5; // the smaller the smoother but more delay
    public static final double MAX_TURN_RATE_DEG = 270.0;   // tốc độ xoay của mục tiêu khi gạt hết cỡ (độ/giây)
    public static final double MAX_LEAD_DEG = 45.0; // mục tiêu được chạy trước heading thật tối đa bấy nhiêu độ
    public static final double TURN_INPUT_DEADBAND = 0.05;  // cần xoay dưới mức này coi như không gạt
    public static final double TURN_FF_POWER = 0.7;
}