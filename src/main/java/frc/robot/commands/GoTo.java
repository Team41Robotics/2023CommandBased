package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Transform2d;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class GoTo extends CommandBase {
        Joystick leftjs, rightjs;
        HDriveSubsystem drive = HDriveSubsystem.getInstance();
        OdomSubsystem odom = OdomSubsystem.getInstance();

        Transform2d target;

        public GoTo(Transform2d target) {
                this.target = target;
                addRequirements(drive);
                leftjs = Robot.leftjs;
                rightjs = Robot.rightjs;
                wPID.enableContinuousInput(-Math.PI, Math.PI);
        }

        PIDController xPID = new PIDController(7, 0, 0);
        PIDController yPID = new PIDController(7, 0, 0);
        PIDController wPID = new PIDController(6, 0, 1);

        @Override
        public void initialize() {
                xPID.reset();
                yPID.reset();
                wPID.reset();
        }

        @Override
        public void execute() {
                if (Math.abs(xPID.getPositionError()) < 0.1) xPID.setI(0.5);
                if (Math.abs(yPID.getPositionError()) < 0.1) yPID.setI(0.5);
                if (Math.abs(wPID.getPositionError()) < 10 * Math.PI / 180) wPID.setI(0.5);

                double vx = xPID.calculate(odom.now().getX(), target.getX());
                double vy = yPID.calculate(odom.now().getY(), target.getY());
                double w = wPID.calculate(odom.now().getTheta(), target.getTheta());

                double robot_angle = odom.now().getTheta();
                double vf = Math.cos(robot_angle) * vx + Math.sin(robot_angle) * vy;
                double vs = -Math.sin(robot_angle) * vx + Math.cos(robot_angle) * vy;
                drive.drive(vf, vs, w);
        }

        public boolean isFinished() {
                return Math.abs(odom.now().getX() - target.getX()) <= 0.03
                                && Math.abs(odom.now().getY() - target.getY()) <= 0.03
                                && Math.abs(odom.now().getTheta() - target.getTheta()) <= 4 / 180. * Math.PI;
        }
}
