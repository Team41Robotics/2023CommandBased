package frc.robot.commands;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ArmPosition;

public class ArmTo extends CommandBase {
	ArmSubsystem arm = ArmSubsystem.getInstance();
	ArmPosition pos;

	TrapezoidProfile elev_prof;
	TrapezoidProfile jt1_prof;
	TrapezoidProfile jt2_prof;

	PIDController elev_pid = new PIDController(1, 0, 0);
	PIDController jt1_pid = new PIDController(1, 0, 0);
	PIDController jt2_pid = new PIDController(1, 0, 0);

	double st = Timer.getFPGATimestamp();

	public ArmTo(ArmPosition pos) {
		this.pos = pos;
		elev_prof = new TrapezoidProfile(new Constraints(ELEV_MAX_SPEED, ELEV_MAX_ACCEL), new State(pos.e, 0));
		jt1_prof = new TrapezoidProfile(new Constraints(JOINT1_MAX_SPEED, JOINT1_MAX_ACCEL), new State(pos.j1, 0));
		jt2_prof = new TrapezoidProfile(new Constraints(JOINT2_MAX_SPEED, JOINT2_MAX_ACCEL), new State(pos.j2, 0));
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

		double elev_fb = elev_pid.calculate(arm.getElevPos(), elevs.position);
		double jt1_fb = jt1_pid.calculate(arm.getJoint1Pos(), jt1s.position);
		double jt2_fb = jt2_pid.calculate(arm.getJoint2Pos(), jt2s.position);

		arm.set(elev_fb + elevs.velocity, jt1_fb + jt1s.velocity, jt2_fb + jt2s.velocity, elev_a, jt1_a, jt2_a);
	}

	@Override
	public void end(boolean interrupted) {
		arm.set(0, 0, 0);
	}

        @Override
        public boolean isFinished() {
		//if (v > 0 && arm.isFwdLimitSwitch()) return true;
		//if (v < 0 && arm.isRevLimitSwitch()) return true;
        }
}
