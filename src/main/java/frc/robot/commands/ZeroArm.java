package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.MechanicalConstants.ArmConstants;

public class ZeroArm extends SequentialCommandGroup {
	static RelativeEncoder enc = arm.jt1.getEncoder();

	// spotless:off
	public ZeroArm() {
		super(
                        // run arm until switch
                        new InstantCommand(() -> arm.elev.set(0)),
                        new InstantCommand(() -> arm.jt1.set(0)),
                        new InstantCommand(() -> arm.jt2.set(.3), arm),
                        new WaitUntilCommand(() -> !arm.joint2_limit.get()),
                        new InstantCommand(() -> arm.jt2.set(0)),

                        // zero
                        new InstantCommand(() -> arm.jt2.getEncoder().setPosition(2.5362667 * ArmConstants.JOINT2_RATIO / 2 / PI)),

                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> arm.jt2.set(-1)), // go back out
                                        new WaitUntilCommand(() -> arm.getJoint2Pos() < PI*75/180.),
                                        new InstantCommand(() -> arm.jt2.setVoltage(ArmConstants.JOINT2_IDENTF.kG * cos(PI/4))),
                                        new WaitCommand(1),
                                        new InstantCommand(() -> arm.jt2.set(-1)), // go back out
                                        new WaitUntilCommand(() -> arm.getJoint2Pos() < PI*55/180.),
                                        new InstantCommand(() -> arm.jt2.setVoltage(ArmConstants.JOINT2_IDENTF.kG * cos(PI/4)))
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(1),
                                        new InstantCommand(() -> arm.jt1.set(.5)), // zero joint 1
                                        new WaitUntilCommand(() -> enc.getVelocity() / 60 * 2 * PI / ArmConstants.JOINT1_RATIO > 0.1),
                                        new WaitUntilCommand(() -> enc.getVelocity() / 60 * 2 * PI / ArmConstants.JOINT1_RATIO < 0.1),

                                        new InstantCommand(() -> enc.setPosition(16.15 / 360. * ArmConstants.JOINT1_RATIO)),
                                        new InstantCommand(() -> arm.jt1.set(0))
                                )
                        )
                        /*
                        */
                );
	}
        // spotless:on
}
