package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDLocations;

public class LEDSubsystem extends SubsystemBase {
	AddressableLED led = new AddressableLED(0);
	AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(108);
	ShuffleboardTab lightTab = Shuffleboard.getTab("LEDS");

	public void initLights() { // perhaps just constructor?
		led.setLength(ledBuffer.getLength());
		led.setData(ledBuffer);
		led.start();
	}

	Color left = Color.kRed, mid = Color.kRed, right = Color.kRed;
	boolean lf = false, mf = false, rf = false;

	int I = 0;

	@Override
	public void periodic() {
		I++;
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			if (i > LEDConstants.LEFT.getFirst() && i < LEDConstants.LEFT.getSecond()) {
				if (I % 10 < 5 && lf) ledBuffer.setLED(i, Color.kBlack);
				else ledBuffer.setLED(i, left);
			}
			if (i > LEDConstants.RIGHT.getFirst() && i < LEDConstants.RIGHT.getSecond()) {
				if (I % 10 < 5 && rf) ledBuffer.setLED(i, Color.kBlack);
				else ledBuffer.setLED(i, right);
			}
			if (i > LEDConstants.MID.getFirst() && i < LEDConstants.MID.getSecond()) {
				if (I % 10 < 5 && mf) ledBuffer.setLED(i, Color.kBlack);
				else ledBuffer.setLED(i, mid);
			}
		}
		led.setData(ledBuffer);
	}

	public void flash(LEDLocations loc, Color c) {
		left = Color.kBlack;
		lf = false;
		right = Color.kBlack;
		rf = false;
		mid = Color.kBlack;
		mf = false;
		if (loc == LEDLocations.LEFT) {
			left = c;
			lf = true;
		}
		if (loc == LEDLocations.RIGHT) {
			right = c;
			rf = true;
		}
		if (loc == LEDLocations.MID) {
			mid = c;
			mf = true;
		}
	}

	static LEDSubsystem instance;

	public static LEDSubsystem getInstance() {
		if (instance == null) {
			instance = new LEDSubsystem();
		}
		return instance;
	}
}
