package frc.robot;

public class SystemConstants {

    public static class  IntakeConstants{
        public static final int pivotMotorId  = 26;
        public static final boolean pivotMotorInverted = false;

        
        public static final double  floorPosition= 48;
        public static final double  feedPosition= 5;
        public static final double  midPosition = 25;

        public static final double pivotPID_P = 0.1;
        public static final double pivotPID_I = 0;
        public static final double pivotPID_D = 0.02;
        public static final double pivotPID_FF = 0;



        public static final int rollerMotorId  = 23;
        public static final boolean rollerMotorInverted = false;

        public static final double rollerHoldVoltage = 1.3;
        public static final double rollerPushVoltage = -3;
        public static final double rollerCaptureVoltage = 3;
        public static final double push4AmpVoltage = -3;





    }



    public static class ShooterConstants{

        public static final int shooterLeftMotorId = 20;
        public static final boolean shooterLeftMotorInverted = false;

        public static final int shooterRightMotorId = 21;
        public static final boolean shooterRightMotorInverted = true;


       public static final double speakerShootingVoltage = 12;
       public static final double ampShootingVoltage = 3 ;             //ayarlanabilir
    }
}
