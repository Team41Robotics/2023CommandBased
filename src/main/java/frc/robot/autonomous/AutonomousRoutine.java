package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoTo;
import frc.robot.commands.ZeroArm;
import frc.robot.util.Transform2d;
import frc.robot.util.Util;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

/**
 * Represents an autonomous routine, and also keeps track with a registry to allow selecting though
 * shuffleboard with an optional [0, 15] second delay
 */
public class AutonomousRoutine {
	/** Tab to configure auto */
	public static final ShuffleboardTab AUTO_TAB = Shuffleboard.getTab("Autonomous");

	/** A chooser to select which auto to run */
	public static final SendableChooser<AutonomousRoutine> AUTO_CHOOSER = new SendableChooser<>();

	/** Chooser to select autonomous delay (seconds) */
	public static final SendableChooser<Double> AUTO_DELAY_CHOOSER = new SendableChooser<>();

	/** HashMap registry to store all the created auto programs */
	private static final Map<String, AutonomousRoutine> AUTO_REGISTRY = new HashMap<>();

	/** Most basic auto already defined */
	public static final AutonomousRoutine DO_NOTHING = new AutonomousRoutine("Do Nothing", () -> new ZeroArm());

	private final AutonomousProvider provider;
	private final String name;
	public final ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

	@SuppressWarnings("unchecked")
	public void dfs_show(ArrayList<Pose2d> poses, Command auton)
			throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {
		if (auton == null) return;
		if (auton instanceof GoTo) {
			Transform2d pose = ((GoTo) auton).target;
			if (DriverStation.getAlliance() == Alliance.Red) pose = Util.flipPoseAcrossField(pose);
			poses.add(new Pose2d(pose.x, pose.y, new Rotation2d(pose.theta)));
		}
		if (auton instanceof SequentialCommandGroup) {
			Field f = SequentialCommandGroup.class.getDeclaredField("m_commands");
			f.setAccessible(true);
			for (Command cmd : (ArrayList<Command>) f.get(auton)) dfs_show(poses, cmd);
		}
		if (auton instanceof ParallelCommandGroup) {
			Field f = ParallelCommandGroup.class.getDeclaredField("m_commands");
			f.setAccessible(true);
			for (Command cmd : ((HashMap<Command, Boolean>) f.get(auton)).keySet()) dfs_show(poses, cmd);
		}
		if (auton instanceof ParallelDeadlineGroup) {
			Field f = ParallelDeadlineGroup.class.getDeclaredField("m_commands");
			f.setAccessible(true);
			for (Command cmd : ((HashMap<Command, Boolean>) f.get(auton)).keySet()) dfs_show(poses, cmd);
		}
		if (auton instanceof ParallelRaceGroup) {
			Field f = ParallelRaceGroup.class.getDeclaredField("m_commands");
			f.setAccessible(true);
			for (Command cmd : (HashSet<Command>) f.get(auton)) dfs_show(poses, cmd);
		}
	}

	private AutonomousRoutine(String name, AutonomousProvider provider) {
		this.provider = provider;
		this.name = name;
		try {
			dfs_show(poses, provider.construct());
		} catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
			e.printStackTrace();
		}
	}

	public String getName() {
		return this.name;
	}

	/**
	 * Creates an instance using the provider
	 *
	 * @return The instaniated base command of the auto (Might be null)
	 */
	public Command construct() {
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
		Command construct();
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
