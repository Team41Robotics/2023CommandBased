package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableDouble implements Sendable {
	public double x;

	public SendableDouble(double x) {
		this.x = x;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Text View");
		builder.addDoubleProperty("x", () -> x, (x) -> this.x = x);
	}

	public int INT() {
		return (int) x;
	}
}
