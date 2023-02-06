package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.HashMap;
import java.util.Map;

/**
 * Represents an autonomous routine, and also keeps track with a registry to allow selecting though
 * shuffleboard with an optional [0, 15] second delay
 */
public class AutonomousRoutine {
	/** Tab to configure auto */
	public static final ShuffleboardTab AUTO_TAB = Shuffleboard.getTab("Autonomous");

	/** A chooser to select which auto to run */
	public static SendableChooser<AutonomousRoutine> AUTO_CHOOSER = new SendableChooser<>();

	/** Chooser to select autonomous delay (seconds) */
	public static final SendableChooser<Double> AUTO_DELAY_CHOOSER = new SendableChooser<>();

	/** HashMap registry to store all the created auto programs */
	private static final Map<String, AutonomousRoutine> AUTO_REGISTRY = new HashMap<>();

	/** Most basic auto already defined */
	public static final AutonomousRoutine DO_NOTHING = new AutonomousRoutine("Do Nothing", () -> null);

	private final AutonomousProvider provider;
	private final String name;

	private AutonomousRoutine(String name, AutonomousProvider provider) {
		this.provider = provider;
		this.name = name;
	}

	public String getName() {
		return this.name;
	}

	/**
	 * Creates an instance using the provider
	 *
	 * @return The instaniated base command of the auto (Might be null)
	 */
	public CommandBase construct() {
		return provider.construct();
	}

	/**
	 * Registers an auto routine
	 *
	 * @param name The name to show in SmartDashboard
	 * @param provider A lambda which returns a command or null
	 */
	public static void create(String name, AutonomousProvider provider) {
		if (AUTO_REGISTRY.get(name) != null)
			throw new IllegalArgumentException(String.format("Duplicate autonomous registered with name \"%s\"", name));

		AUTO_REGISTRY.put(name, new AutonomousRoutine(name, provider));
	}

	/**
	 * Basic Lambda for creating new command instances (so that autos that dont clean up arent broken)
	 *
	 * @return
	 */
	interface AutonomousProvider {
		CommandBase construct();
	}

	/** Adds all values from registry to Shuffleboard */
	private static void addAutosToShuffleboard() {
		Autonomous.initAutos();
		// Add commands to the autonomous command chooser
		for (var auto : AUTO_REGISTRY.values()) AUTO_CHOOSER.addOption(auto.getName(), auto);

		System.out.println(AUTO_REGISTRY);

		AUTO_CHOOSER.setDefaultOption(DO_NOTHING.getName(), DO_NOTHING);

		// Put the chooser on the dashboard
		AUTO_TAB.add("Autonomous Selector", AUTO_CHOOSER);
	}

	public static void initShuffleboard() {
		/* Add basic auto to registry */
		AUTO_REGISTRY.put(DO_NOTHING.name, DO_NOTHING);

		/* Defines all the options for the autonomous delay */
		for (double i = 0; i < 15; i += 0.25) AUTO_DELAY_CHOOSER.addOption(String.format("%.2f", i), i);

		AUTO_DELAY_CHOOSER.setDefaultOption("0.0", 0.0D);

		AUTO_TAB.add("Auto Start Delay", AUTO_DELAY_CHOOSER);

		/* Add routines to shuffleboard */
		addAutosToShuffleboard();
	}
}
