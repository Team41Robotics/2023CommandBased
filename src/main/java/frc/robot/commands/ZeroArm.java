package frc.robot.commands;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroArm extends CommandBase {
	ArmSubsystem arm = ArmSubsystem.getInstance();

	public ZeroArm() {
		addRequirements(arm);
	}

	boolean prev_above = false;

	@Override
	public void execute() {
		if (arm.jt1.getEncoder().getVelocity() / 60. * 2 * PI / JOINT1_RATIO > JOINT1_ZERO_THRES) prev_above = true;
		if (arm.jt1.getEncoder().getVelocity() / 60. * 2 * PI / JOINT1_RATIO < JOINT1_ZERO_THRES && prev_above)
			arm.jt1.getEncoder().setPosition(JOINT1_ZERO_ANGLE / 2 / PI * JOINT1_RATIO);
		arm.jt1.set(0.2);
		arm.jt2.set(-0.2);
	}

	@Override
	public boolean isFinished() {
		return arm.jt1.getEncoder().getVelocity() / 60. * 2 * PI / JOINT1_RATIO < JOINT1_ZERO_THRES && prev_above;
	}
}
