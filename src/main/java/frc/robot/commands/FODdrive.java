package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class FODdrive extends CommandBase {
        Joystick leftjs, rightjs;
        HDriveSubsystem drive = HDriveSubsystem.getInstance();
        OdomSubsystem odom = OdomSubsystem.getInstance();

        public FODdrive() {
                addRequirements(drive);
                leftjs = Robot.leftjs;
                rightjs = Robot.rightjs;
        }

        public void execute() {
                double vf = -Util.deadZone(leftjs.getY());
                double vs = -Util.deadZone(leftjs.getX());
                double w = -Util.deadZone(rightjs.getX());
                double robot_angle = odom.now().getTheta();
                double vx = Math.cos(robot_angle) * vf + Math.sin(robot_angle) * vs;
                double vy = -Math.sin(robot_angle) * vf + Math.cos(robot_angle) * vs;
                drive.drive(vx, vy, w);
        }

        @Override
        public boolean isFinished() {
                return true;
                // trivial
        }
}
