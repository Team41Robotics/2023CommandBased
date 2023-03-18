package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.LEDSubsystem.LEDSegment.*;

public class LEDSubsystem extends SubsystemBase {
	public static int timeStep = 0;
	public static AddressableLED led = new AddressableLED(0);
	public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(108);
	ShuffleboardTab lightTab = Shuffleboard.getTab("LEDS");

	public LEDSubsystem() { // perhaps just constructor?
		led.setLength(ledBuffer.getLength());
		led.setData(ledBuffer);
		led.start();
		rightSide.setColor(Color.kDarkBlue);
		midSide.setColor(Color.kDarkRed);
		leftSide.setColor(Color.kYellow);
	//	midSide.setRainbow();
	//	leftSide.setRainbow();
	}

	Color left = Color.kRed, mid = Color.kRed, right = Color.kRed;
	boolean lf = false, mf = false, rf = false;

	int I = 0;

	@Override
	public void periodic() {
		timeStep++;
		leftSide.showLEDS();
		rightSide.showLEDS();
		midSide.showLEDS();

		led.setData(ledBuffer);
	}

	public void flash(LEDSegment loc, Color c) {
		leftSide.disable();
		LEDSegment.rightSide.disable();
		LEDSegment.midSide.disable();
		if (loc == null) return;
		loc.flashColor(c);
	}

	static LEDSubsystem instance;

	public static LEDSubsystem getInstance() {
		if (instance == null) {
			instance = new LEDSubsystem();
		}
		return instance;
	}

	public static enum ledModes {
		SET_COLOR,
		RAINBOW,
		FLICKER
	}

	public static enum LEDSegment {
		leftSide(0, 45, ledBuffer),
		midSide(45, 20, ledBuffer),
		rightSide(66, 40, ledBuffer),
		;
		public final int startIndex;
		public final int segmentSize;
		public final AddressableLEDBuffer buffer;
		private ledModes mode = ledModes.SET_COLOR;
		public Color color = Color.kRed;

		private LEDSegment(int startIndex, int segmentSize, AddressableLEDBuffer buffer) {
			this.startIndex = startIndex;
			this.segmentSize = segmentSize;
			this.buffer = buffer;
		}

		public void flashColor(Color c) {
			this.mode = ledModes.FLICKER;
			this.color = c;
		}

		public void setColor(Color c) {
			this.mode = ledModes.SET_COLOR;
			this.color = c;
		}

		public void disable() {
			setColor(Color.kBlack);
		}

		public void setRainbow() {
			this.mode = ledModes.RAINBOW;
		}

		private double hangTime = 0;
		private double offset;
		private final double hangTimeMax = 1;

		private void rainbow() {
			int length = buffer.getLength();
			for (var i = startIndex; i < startIndex + segmentSize; i++) {
				//if (i < startIndex || i > startIndex + segmentSize) continue;
				int hue = 0;
				if (i < Math.abs(offset - length / 2)) {
					buffer.setRGB(i, 0, 0, 0);

					//buffer.setRGB(-i + length - 1, 0, 0, 0);
				} else {
					hue = ((i * 180 / buffer.getLength() * 2)) % 180;
					buffer.setHSV(i, hue, 255, 128);
					//buffer.setHSV(-i + buffer.getLength() - 1, hue, 255, 128);
				}
			}

			if (hangTime + hangTimeMax <= Timer.getFPGATimestamp()) {
				offset = (offset + 0.25) % 108;
				if (offset == 54) hangTime = Timer.getFPGATimestamp();
			}
		}

		private void flicker() {
			for (int j = startIndex; j <= startIndex + segmentSize; j++) {
				buffer.setLED(j, (timeStep % 20 >= 7 ? color : Color.kBlack));
			}
		}

		private void solidColor() {
			for (int j = startIndex; j <= startIndex + segmentSize; j++) {
				buffer.setLED(j, color);
			}
		}

		public void showLEDS() {
			switch (mode) {
				case RAINBOW:
					rainbow();
					break;
				case FLICKER:
					flicker();
					break;
				case SET_COLOR:
					solidColor();
					break;
			}
		}
	}
}
