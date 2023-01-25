package frc.robot.commands;

import frc.robot.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class GoTo extends CommandBase {
        Joystick leftjs, rightjs;
        HDriveSubsystem drive = HDriveSubsystem.getInstance();
        OdomSubsystem odom = OdomSubsystem.getInstance();

        Matrix target;
        public GoTo(Matrix target) {
                this.target = target;
                addRequirements(drive);
                leftjs = Robot.leftjs;
                rightjs = Robot.rightjs;
                wPID.enableContinuousInput(-Math.PI, Math.PI);
        }

        PIDController xPID = new PIDController(7,0,0);
        PIDController yPID = new PIDController(7,0,0);
        PIDController wPID = new PIDController(7,0,0);

        @Override
        public void initialize() {
                xPID.reset();
                yPID.reset();
                wPID.reset();
        }

        @Override
        public void execute() {
                if(Math.abs(xPID.getPositionError()) < 0.1) xPID.setI(0);
                if(Math.abs(yPID.getPositionError()) < 0.1) yPID.setI(0);
                if(Math.abs(wPID.getPositionError()) < 10*Math.PI/180) wPID.setI(0);

                double vf = xPID.calculate(odom.now().getX(), target.getX());
                double vs = yPID.calculate(odom.now().getY(), target.getY());
                double w = wPID.calculate(odom.now().getTheta(), target.getTheta());

                double robot_angle = Robot.imu.getYaw();
                double vx =  Math.cos(robot_angle) * vf + Math.sin(robot_angle) * vs;
                double vy = -Math.sin(robot_angle) * vf + Math.cos(robot_angle) * vs;
                drive.drive(vx, vy, w);
                //System.out.println("x: " + tgt_in_robot.getX() + " y: " + tgt_in_robot.getY() + " theta: " + tgt_in_robot.getTheta()*180/Math.PI);
                //System.out.println("vx: " + vx + " vy: " + vy + " w: " + w);
        }
        public boolean isFinished() {
                return Math.abs(odom.now().getX() - target.getX())<=0.03
                        && Math.abs(odom.now().getY() - target.getY())<=0.03
                        && Math.abs(odom.now().getTheta() - target.getTheta())<=10/180.*Math.PI;
        }
}
