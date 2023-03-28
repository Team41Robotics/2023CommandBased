package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.*;
import frc.robot.constants.MechanicalConstants.ArmConstants;
import frc.robot.util.ArmPosition;

public class ArmTo extends CommandBase { // TODO future trapezoid profile?
	ArmPosition pos;

	public ArmTo(ArmPos pos) {
		this(pos.asPostion());
	}

	public ArmTo(ArmPosition pos) {
		addRequirements(arm);

		this.pos = pos;
	}

	@Override
	public void execute() {
		arm.set(pos.e, pos.j1, pos.j2, 0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		return abs(arm.elev_pid.getPositionError()) < ArmConstants.ELEVATOR_TOLERANCE
				&& abs(arm.jt1_pid.getPositionError()) < ArmConstants.JOINT_TOLERANCE
				&& abs(arm.jt2_pid.getPositionError()) < ArmConstants.JOINT_TOLERANCE;
	}
}
