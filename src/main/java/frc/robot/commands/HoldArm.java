package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.ArmPosition;

public class HoldArm extends CommandBase {
	ArmPosition pos;

	public HoldArm() {
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		pos = new ArmPosition(arm.getElevPos(), arm.getJoint1Pos(), arm.getJoint2Pos());
	}

	@Override
	public void execute() {
		arm.set(pos.e, pos.j1, pos.j2, 0, 0, 0);
	}
}
