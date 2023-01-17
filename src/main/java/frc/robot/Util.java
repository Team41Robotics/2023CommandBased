package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Util {
        public static double deadZone(double joystickAxis){
                return Math.abs(joystickAxis) > 0.2 ? joystickAxis : 0;
        }
        public static Transform2d transformThreeToTwo(Transform3d trans) {
                return new Transform2d(trans.getTranslation().toTranslation2d(),trans.getRotation().toRotation2d());
        }
}