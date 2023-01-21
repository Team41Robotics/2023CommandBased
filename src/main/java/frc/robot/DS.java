package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DS {
        public static Joystick left_js = new Joystick(1);
        public static Joystick right_js = new Joystick(0);
        // X + when back
        // Y + when right
        
        public static double getLX() {return left_js.getX();}
        public static double getLY() {return left_js.getY();}
        public static double getRX() {return right_js.getX();}
        public static double getRY() {return right_js.getY();}
        public static boolean getLThumb() {return left_js.getRawButton(2);}
        public static boolean getRThumb() {return right_js.getRawButton(2);}
        public static boolean getLTrig() {return left_js.getRawButton(1);}
        public static boolean getRTrig() {return right_js.getRawButton(1);}
}
