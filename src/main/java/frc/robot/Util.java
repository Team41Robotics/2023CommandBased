package frc.robot;

public class Util {
    public static double deadZone(double joystickAxis) {
        return Math.abs(joystickAxis) > 0.2 ? joystickAxis : 0;
    }

    public static double normRot(double rad) {
        rad %= 2 * Math.PI;
        rad += 2 * Math.PI;
        rad %= 2 * Math.PI;
        if (rad > Math.PI) rad -= 2 * Math.PI;
        return rad;
    }
}
