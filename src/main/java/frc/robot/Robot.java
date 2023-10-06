package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ArmPos;

import java.util.ArrayList;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.ArmPos.*;
import static frc.robot.constants.Constants.isCone;
import static java.lang.Math.PI;

public class Robot extends TimedRobot {
    public boolean FOD;
    public Command autonomousCommand;
    public Field2d autonField = new Field2d();
    boolean has_been_enabled = false;

    public void schedule(Command cmd) {
        CommandScheduler.getInstance().schedule(cmd);
    }

    @Override
    public void robotInit() {
        robot = this;
        initSubsystems();
        configureButtons();

        Shuffleboard.getTab("Drivetrain").addBoolean("FOD", () -> FOD);
        hdrive.setDefaultCommand(new ConditionalCommand(new FODdrive(), new Drive(), () -> FOD));
        arm.setDefaultCommand(new HoldArm());
        AutonomousRoutine.AUTO_TAB.add(autonField);
        AutonomousRoutine.initShuffleboard();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        ArrayList<Pose2d> poses = AutonomousRoutine.AUTO_CHOOSER.getSelected().poses;
        autonField.setRobotPose(odom.now().x, odom.now().y, new Rotation2d(odom.now().theta));
        if (DriverStation.getAlliance() == Alliance.Red)
            poses.replaceAll(pose2d -> new Pose2d(
                    Constants.FIELD_LENGTH - pose2d.getX(),
                    pose2d.getY(),
                    new Rotation2d(PI - pose2d.getRotation().getRadians())));
        autonField.getObject("Robt").setPoses(poses);
        if (DriverStation.getAlliance() == Alliance.Red)
            poses.replaceAll(pose2d -> new Pose2d(
                    Constants.FIELD_LENGTH - pose2d.getX(),
                    pose2d.getY(),
                    new Rotation2d(PI - pose2d.getRotation().getRadians())));
    }

    public void enabledInit() {
        if (has_been_enabled) return;
        has_been_enabled = true;
        odom.start();
    }

    @Override
    public void autonomousInit() {
        enabledInit();

        autonomousCommand = AutonomousRoutine.AUTO_CHOOSER.getSelected().construct();
        double delay = AutonomousRoutine.AUTO_DELAY_CHOOSER.getSelected();
        if (autonomousCommand != null) {
            SequentialCommandGroup cmd = new SequentialCommandGroup(new WaitCommand(delay), autonomousCommand);
            schedule(cmd);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        enabledInit();
        leds.allRainbow();

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        // arm.set(.5,0,0,0,0,0,0,0,0);
        arm.hold();
    }

    public ArmPos getCurrentArm() {
        System.out.println(operator.queuedValue);
        int x = (operator.queuedValue != null ? operator.queuedValue.x : 0);
        int y = (operator.queuedValue != null ? operator.queuedValue.y : 0);
        System.out.println(x);
        if (x == 2) return ALL_BOT;
        System.out.println(ArmPos.values()[x + (isCone[x] ? 0 : 3)].name());
        return ArmPos.values()[x + (isCone[y] ? 3 : 0)];
    }

    public void configureButtons() { // TODO remap
        // new JoystickButton(DS, 1).onTrue(new InstantCommand(operator::setPiece));

        // new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));

        controller.leftTrigger().onTrue(new RunIntake(.6, true));
        controller.rightTrigger().onTrue(new RunIntake(-.6, true));
        controller.leftBumper().onTrue(new RunIntake(0));
        controller
                .start()
                .whileTrue(new HoldArm().repeatedly().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        controller.x().onTrue(new ArmTo(BALL_TOP));
        controller.y().onTrue(new ArmTo(CONE_TOP));
        controller.b().onTrue(new ArmTo(BALL_PICKUP));
        controller.a().onTrue(new ArmTo(CONE_PICKUP));
        controller.rightBumper().onTrue(new ArmTo(BALL_SLIDE));

        // new JoystickButton(DS, 2).onTrue(new Balance().until(() -> rightjs.getRawButton(3)));
        // new JoystickButton(DS, 5).whileTrue(new MovArm(0, -0.1, 1));
        // new JoystickButton(DS, 6).whileTrue(new MovArm(0, 0.1, 1));

    }
}
