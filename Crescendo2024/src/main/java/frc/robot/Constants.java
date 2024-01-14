package frc.robot;

public final class Constants {
    public static class DriveMotorConstants {

        public static final int LEFT_DRIVE1_ID = 1;
        public static final int LEFT_DRIVE2_ID = 2;
        public static final int RIGHT_DRIVE1_ID = 3;
        public static final int RIGHT_DRIVE2_ID = 4;

        public static final int CURRENT_LIMIT = 30;
        
        public static final int KP = 0;
        public static final int KI = 0;
        public static final int KD = 0;
    }

    public static class ControlConstants {
        public static final int XBOX_CONTROLLER_DRIVER = 0;
        public static final int XBOX_CONTROLLER_OPERATOR = 1;
        public static final double AXIS_THRESHOLD = 0.05;
        public static final double SPIN_SENSITIVITY = 0.8;
    }
    public static class ArmMotorConstants {
        public static final int ARM_DRIVE_ID = 5;
    }
}
