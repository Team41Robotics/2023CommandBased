package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.IMU;
import frc.robot.commands.Drive;
import frc.robot.commands.FODdrive;
import frc.robot.subsystems.HDriveSubsystem;
public class Robot extends TimedRobot {
        private Command autonomousCommand;
        public static Joystick leftJoy = new Joystick(0);
        public static Joystick rightJoy = new Joystick(1);
        public static IMU imu = new IMU();
        @Override
        public void robotInit() {
        }

        @Override
        public void robotPeriodic() {
                CommandScheduler.getInstance().run();
                HDriveSubsystem.getInstance().setDefaultCommand(new FODdrive());
        }

        @Override
        public void autonomousInit() {
        }

        @Override
        public void autonomousPeriodic() {
        }

        @Override
        public void teleopInit() {
                if (autonomousCommand != null) {
                        autonomousCommand.cancel();
                }
        }

        @Override
        public void teleopPeriodic() {
        }
}
