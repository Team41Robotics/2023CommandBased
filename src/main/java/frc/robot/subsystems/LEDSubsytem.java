package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.lightPositions;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LEDLocations;

public class LEDSubsytem extends SubsystemBase {
	AddressableLED m_led = new AddressableLED(0);
	AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(108);
	int m_rainbowFirstPixelHue;
	ShuffleboardTab lightTab = Shuffleboard.getTab("LEDS");
	double offset = 0;
	Joystick testjs = new Joystick(0);
	private long start;

	public void initLights() { // perhaps just constructor?
		m_led.setLength(m_ledBuffer.getLength());
		start = System.currentTimeMillis();
		bootUp();
		m_led.setData(m_ledBuffer);
		m_led.start();
	}

	private void loadBuffer() {
		flicker = (flicker + 1) % 20;
		if (System.currentTimeMillis() - start <= 1000) {
			bootUp();
			return;
		}
		if (DriverStation.isEnabled()) rainbow();
	}

	@Override
	public void periodic() {
		if (DriverStation.isEnabled()) {
			for (int i = 0; i < m_ledBuffer.getLength(); i++) {
				m_ledBuffer.setLED(i, Color.kBlack);
			}
			loadBuffer();
		}

		m_led.setData(m_ledBuffer);
	}

	public void setColor(LEDLocations location, Color color) {
		int start = lightPositions.get(location).getFirst();
		int length = lightPositions.get(location).getSecond();
		for (int i = start; i < start + length - 1; i++) m_ledBuffer.setLED(i, color);
	}

	public void flash(LEDLocations location, Color color) {
		int start = lightPositions.get(location).getFirst();
		int length = lightPositions.get(location).getSecond();
		for (int i = start; i < start + length; i++) m_ledBuffer.setLED(i, (flicker > 7 ? color : Color.kBlack));
	}

	private void bootUp() {
		for (int i = 0; i < m_ledBuffer.getLength(); i++) {
			m_ledBuffer.setLED(i, Color.kDarkRed);
		}
	}

	static int flicker;

	private void flashRight() {
		for (int i = 0; i < 45; i++) m_ledBuffer.setLED(i, (flicker > 7 ? Color.kPurple : Color.kBlack));

		flicker = (flicker + 1) % 20;
	}

	private double hangTime = 0;
	private final double hangTimeMax = 1;

	private void rainbow() {
		int length = m_ledBuffer.getLength();
		for (var i = 0; i < length / 2; i++) {
			int hue = 0;
			if (i < Math.abs(offset - length / 2)) {
				m_ledBuffer.setRGB(i, 0, 0, 0);

				m_ledBuffer.setRGB(-i + m_ledBuffer.getLength() - 1, 0, 0, 0);
			} else {
				hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength() * 2)) % 180;
				m_ledBuffer.setHSV(i, hue, 255, 128);
				m_ledBuffer.setHSV(-i + m_ledBuffer.getLength() - 1, hue, 255, 128);
			}
		}

		if (hangTime + hangTimeMax <= Timer.getFPGATimestamp()) {
			offset = (offset + 0.25) % 108;
			if (offset == 54) hangTime = Timer.getFPGATimestamp();
		}
		m_rainbowFirstPixelHue %= 180;
	}

	static LEDSubsytem instance;

	public static LEDSubsytem getInstance() {
		if (instance == null) {
			instance = new LEDSubsytem();
		}
		return instance;
	}
}
