package frc.robot.util;

import static java.lang.Math.*;

public class SystemIdentification {
	public final double kG;
	public final double kS;
	public final double kV;
	public final double kA;

	public SystemIdentification(double kG, double kS, double kV, double kA) {
		this.kG = kG;
		this.kS = kS;
		this.kV = kV;
		this.kA = kA;
	}

	public double getOutput(double Gfactor, double vel, double a) {
		return Gfactor * kG + signum(vel) * kS + vel * kV + a * kA;
	}
}
