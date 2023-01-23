package frc.robot.commands; 

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.Robot;
import frc.robot.Util;

public class FODdrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    Joystick leftJoy,rightJoy;
    HDriveSubsystem drive = HDriveSubsystem.getInstance();

    public FODdrive(){
        addRequirements(drive);
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
    }
    public void execute(){
        double vf = -Util.deadZone(leftJoy.getY());
        double vs = -Util.deadZone(leftJoy.getX());
        double omega = -Util.deadZone(rightJoy.getX());
        double robot_angle = Robot.imu.getYaw();
        double vx = vf * Math.cos(robot_angle) - vs * Math.sin(robot_angle);
        double vy = vf * Math.sin(robot_angle) + vs * Math.cos(robot_angle);
        drive.drive(vx, vy, omega);
    }

}