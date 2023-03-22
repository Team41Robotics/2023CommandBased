package frc.robot.commands;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroArm extends CommandBase {
	public ZeroArm() {
		addRequirements(arm);
	}

	boolean prev_above = false;

	@Override
	public void execute() {
		/* =
		if (arm.jt1.getEncoder().getVelocity() / 60. * 2 * PI / JOINT1_RATIO > JOINT1_ZERO_THRES) prev_above = true;
		if (arm.jt1.getEncoder().getVelocity() / 60. * 2 * PI / JOINT1_RATIO < JOINT1_ZERO_THRES && prev_above)
			arm.jt1.getEncoder().setPosition(JOINT1_ZERO_ANGLE / 2 / PI * JOINT1_RATIO);
		arm.jt1.set(0.4);
		if (arm.getJoint2Pos() > 0) arm.jt2.set(-1);
		else arm.jt2.setVoltage(JOINT2_kG);
		*/
	}

	@Override
	public boolean isFinished() {
		return true;
		// return arm.jt1.getEncoder().getVelocity() / 60. * 2 * PI / JOINT1_RATIO < JOINT1_ZERO_THRES && prev_above;
	}
}
