package frc.robot;

public final class Constants {
        public static class OperatorConstants {

                public static final int DRIVER_CONTROLLER_PORT = 0;
                public static final int LEFT_JOYSTICK_PORT = 1;
                public static final int RIGHT_JOYSTICK_PORT = 0;
        }

        public static class DrivetrainConstants {

                public static final int BACK_LEFT_MOTOR_CHANNEL = 0;
                public static final int BACK_RIGHT_MOTOR_CHANNEL = 4;
                public static final int FRONT_LEFT_MOTOR_CHANNEL = 1;
                public static final int FRONT_RIGHT_MOTOR_CHANNEL = 3;
                public static final int MID_MOTOR_CHANNEL = 2;
        }

        public static class ArmConstants {

                public static final int NEO_ONE_DEVICE_ID = 1;
                public static final int NEO_TWO_DEVICE_ID = 2;
                public static final int NEO_THREE_DEVICE_ID = 3;

                public static final double kP = 0.1;
                public static final double kI = 0;
                public static final double kD = 1;
                
                public static final double JOINT_RADIUS;

        }

}
