package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Matrix2d;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class GoTo extends CommandBase {
        Joystick leftjs, rightjs;
        HDriveSubsystem drive = HDriveSubsystem.getInstance();
        OdomSubsystem odom = OdomSubsystem.getInstance();

        Matrix2d target;
        public GoTo(Matrix2d target) {
                this.target = target;
                addRequirements(drive);
                leftjs = Robot.leftjs;
                rightjs = Robot.rightjs;
                wPID.enableContinuousInput(-Math.PI, Math.PI);
                wPID.setIntegratorRange(-10/180.*Math.PI, 10/180.*Math.PI);
                xPID.setIntegratorRange(-0.1, 0.1);
                yPID.setIntegratorRange(-0.1, 0.1);
        }

        PIDController xPID = new PIDController(7,0.5,0);
        PIDController yPID = new PIDController(7,0.5,0);
        PIDController wPID = new PIDController(3,0.5,0);

        Matrix2d tgt_in_robot;
        @Override
        public void execute() {
                tgt_in_robot = odom.now().inverse().mul(target);
                double vx = xPID.calculate(0, tgt_in_robot.getX());
                double vy = yPID.calculate(0, tgt_in_robot.getY());
                double w = wPID.calculate(0, tgt_in_robot.getTheta());
                drive.drive(vx, vy, w);
                System.out.println("x: " + tgt_in_robot.getX() + " y: " + tgt_in_robot.getY() + " theta: " + tgt_in_robot.getTheta()*180/Math.PI);
                System.out.println("vx: " + vx + " vy: " + vy + " w: " + w);
        }
        public boolean isFinished() {
                return Math.abs(tgt_in_robot.getX())<=0.03
                       && Math.abs(tgt_in_robot.getY())<=0.03
                       && Math.abs(tgt_in_robot.getTheta())<=10/180.*Math.PI;
        }
}
