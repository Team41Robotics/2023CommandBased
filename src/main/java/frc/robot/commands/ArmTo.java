package frc.robot.commands;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ArmPosition;

public class ArmTo extends CommandBase {
	static ArmSubsystem arm = ArmSubsystem.getInstance();
	ArmPosition pos;

	TrapezoidProfile elev_prof;
	TrapezoidProfile jt1_prof;
	TrapezoidProfile jt2_prof;

	PIDController elev_pid = new PIDController(1, 0.5, 0);
	PIDController jt1_pid = new PIDController(1, 0.2, 0);
	PIDController jt2_pid = new PIDController(1, 0.2, 0);

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
		elev_prof = new TrapezoidProfile(new Constraints(ELEV_MAX_SPEED, ELEV_MAX_ACCEL), new State(pos.e, 0));
		jt1_prof = new TrapezoidProfile(new Constraints(JOINT1_MAX_SPEED, JOINT1_MAX_ACCEL), new State(pos.j1, 0));
		jt2_prof = new TrapezoidProfile(new Constraints(JOINT2_MAX_SPEED, JOINT2_MAX_ACCEL), new State(pos.j2, 0));
		elev_pid.reset();
		jt1_pid.reset();
		jt2_pid.reset();
	}

	@Override
	public void execute() {
		State elevs = elev_prof.calculate(Timer.getFPGATimestamp() - st);
		State jt1s = jt1_prof.calculate(Timer.getFPGATimestamp() - st);
		State jt2s = jt2_prof.calculate(Timer.getFPGATimestamp() - st);

		double dt = 1e-6;
		double elev_a = (elev_prof.calculate(Timer.getFPGATimestamp() - st + dt).velocity - elevs.velocity) / dt;
		double jt1_a = (jt1_prof.calculate(Timer.getFPGATimestamp() - st + dt).velocity - jt1s.velocity) / dt;
		double jt2_a = (jt2_prof.calculate(Timer.getFPGATimestamp() - st + dt).velocity - jt2s.velocity) / dt;

		double elev_fb = elev_pid.calculate(arm.getElevPos(), elevs.position);
		double jt1_fb = jt1_pid.calculate(arm.getJoint1Pos(), jt1s.position);
		double jt2_fb = jt2_pid.calculate(arm.getJoint2Pos(), jt2s.position);

		double vel = elevs.velocity + elev_fb;
		if (arm.isTopLimitSwitch() && vel > 0) vel = 0;
		if (arm.isBotLimitSwitch() && vel < 0) vel = 0;
		arm.set(vel, jt1_fb + jt1s.velocity, jt2_fb + jt2s.velocity, elev_a, jt1_a, jt2_a);
	}

	@Override
	public void end(boolean interrupted) {
		arm.set(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		if (elev_prof.isFinished(Timer.getFPGATimestamp() - st)
				&& jt1_prof.isFinished(Timer.getFPGATimestamp() - st)
				&& jt2_prof.isFinished(Timer.getFPGATimestamp() - st)) {
			if (Math.abs(elev_pid.getPositionError()) < ELEVATOR_TOLERANCE
					&& Math.abs(jt1_pid.getPositionError()) < JOINT_TOLERANCE) return true;
		}
		return false;
	}
}
