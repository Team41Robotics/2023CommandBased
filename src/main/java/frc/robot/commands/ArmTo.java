package frc.robot.commands;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import static frc.robot.RobotContainer.*;
import static frc.robot.constants.MechanicalConstants.ArmConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.util.ArmPosition;

public class ArmTo extends CommandBase {
	ArmPosition pos;

	TrapezoidProfile elev_prof;
	TrapezoidProfile jt1_prof;
	TrapezoidProfile jt2_prof;

	double st;

	public ArmTo(String pos) {
		this(arm.positions.get(pos));
	}

	public ArmTo(ArmPosition pos) {
		addRequirements(arm);

		this.pos = pos;
	}

	@Override
	public void initialize() {
		st = Timer.getFPGATimestamp();
		elev_prof = new TrapezoidProfile(ELEV_CONSTRAINTS, new State(pos.e, 0));
		jt1_prof = new TrapezoidProfile(JOINT1_CONSTRAINTS, new State(pos.j1, 0));
		jt2_prof = new TrapezoidProfile(JOINT2_CONSTRAINTS, new State(pos.j2, 0));
	}

	@Override
	public void execute() {
		State elevs = elev_prof.calculate(Timer.getFPGATimestamp() - st + Constants.LOOP_TIME);
		State jt1s = jt1_prof.calculate(Timer.getFPGATimestamp() - st + Constants.LOOP_TIME);
		State jt2s = jt2_prof.calculate(Timer.getFPGATimestamp() - st + Constants.LOOP_TIME);

		double dt = 1e-6;
		double elev_a = (elev_prof.calculate(Timer.getFPGATimestamp() - st + dt).velocity - elevs.velocity) / dt;
		double jt1_a = (jt1_prof.calculate(Timer.getFPGATimestamp() - st + dt).velocity - jt1s.velocity) / dt;
		double jt2_a = (jt2_prof.calculate(Timer.getFPGATimestamp() - st + dt).velocity - jt2s.velocity) / dt;

		double vel = elevs.velocity;
		if (arm.isTopLimitSwitch() && vel > 0) vel = 0;
		if (arm.isBotLimitSwitch() && vel < 0) vel = 0;
		arm.set(vel, jt1s.velocity, jt2s.velocity, elev_a, jt1_a, jt2_a);
	}

	@Override
	public void end(boolean interrupted) {
		arm.hold();
	}

	@Override
	public boolean isFinished() {
		if (elev_prof.isFinished(Timer.getFPGATimestamp() - st)
				&& jt1_prof.isFinished(Timer.getFPGATimestamp() - st)
				&& jt2_prof.isFinished(Timer.getFPGATimestamp() - st)) {
			//if (Math.abs(elev_pid.getPositionError()) < ELEVATOR_TOLERANCE
					//&& Math.abs(jt1_pid.getPositionError()) < JOINT_TOLERANCE) return true;
                        return true;
		}
		return false;
	}
}
