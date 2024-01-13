package frc.robot;

public class Constants {
    // PWM
    public static final int LEFTWHEEL = 0;
    public static final int RIGHTWHEEL = 7;
    public static final int liftMotor = 2;
    public static final int grabberMotor = 3;
    // DIO
    public static final int LEFTMOTORENCODERONE = 0;
    public static final int LEFTMOTORENCODERTWO = 1;
    public static final int RIGHTMOTORENCODERONE = 2;
    public static final int RIGHTMOTORENCODERTWO = 3;

    public static final boolean REVERSE_LIFT = true;

    // SPEEDS
    public static final double MOTORSPEED = 0.25;

    public static final double MOTORENCODERTPR = 28;
    // public static final double MOTOR_GEAR_RATIO = 5 / 1; // 5:1
    public static final double MOTORAXELRADIUS = 44.45;

    public static final double MOVE_DIST_FT = 8.7d;    
    public static final double LIFT_TIME = 1d;    

    public static final double LEFTBIAS = 1;

    public static final int SMOOTHDURATION = 19;

    public static final double DEAD_ZONE = 0.5;
}
