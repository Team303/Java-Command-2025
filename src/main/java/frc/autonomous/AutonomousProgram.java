package frc.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.awt.Point;
import java.util.Set;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot.FieldPosition;
import frc.robot.util.ReefState;
import static frc.robot.Robot.swerve;
import static frc.robot.Robot.operatorControl;
import static frc.modules.OperatorControlModule.onTheFlyAutoStart;
import static frc.modules.OperatorControlModule.autoQueue;


public class AutonomousProgram {

	/* Shuffleboard */
	public static final ShuffleboardTab AUTO_TAB = Shuffleboard.getTab("Autonomous");
	public static SendableChooser<AutonomousProgram> autoChooser = new SendableChooser<>();
	public static SendableChooser<Double> autoDelayChooser = new SendableChooser<>();
	public static long[][] nodeStateValues = new long[12][4];


	static {
		/**
		 * Defines all the options for the autonomous delay
		 */
		for (double i = 0; i < 15; i += 0.25)
			autoDelayChooser.addOption(String.format("%.2f", i), i);

		autoDelayChooser.setDefaultOption("0.0", 0.0D);

		AUTO_TAB.add("Auto Start Delay", autoDelayChooser);
	}

	// HashMap registry to store all the created auto programs
	private static Map<String, AutonomousProgram> autoRegistry = new HashMap<>();

	// Most basic auto already defined
	public static AutonomousProgram DO_NOTHING = new AutonomousProgram("Do Nothing", () -> null);

	static {
		// Add basic auto to registry
		autoRegistry.put(DO_NOTHING.name, DO_NOTHING);
	}

	AutonomousProvider provider;
	String name;

	private AutonomousProgram(String name, AutonomousProvider provider) {
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
	public Command construct() {
		return provider.construct();
	}

	/**
	 * Registers an auto program
	 * 
	 * @param name     The name to show in SmartDashboard
	 * @param provider A lambda which returns a command or null
	 */
	public static void create(String name, AutonomousProvider provider) {
		if (autoRegistry.get(name) != null)
			throw new IllegalArgumentException(
					String.format("Duplicate autonomous registered with name \"%s\"", name));

		autoRegistry.put(name, new AutonomousProgram(name, provider));
	}

	/**
	 * Basic Lambda for creating new command instances
	 * (so that autos that dont clean up arent broken)
	 * 
	 * @return
	 */
	interface AutonomousProvider {
		Command construct();
	}

	/**
	 * Adds all values from registry to Shuffleboard
	 */
	public static void addAutosToShuffleboard() {
		// Add commands to the autonomous command chooser
		for (var auto : autoRegistry.values())
			autoChooser.addOption(auto.getName(), auto);

		autoChooser.setDefaultOption(DO_NOTHING.getName(), DO_NOTHING);

		// Put the chooser on the dashboard
		AUTO_TAB.add("Autonomous Selector", autoChooser);
	}

	public static Command constructSelectedRoutine() {
		if(onTheFlyAutoStart.getBoolean(false)){
			Command[] commandArray = new Command[autoQueue.size()];
			for(int i=0;i<autoQueue.size();i++) {
				System.out.println(autoQueue.get(i));
				FieldPosition position = operatorControl.getQueuedPosition(autoQueue.get(i).x);
				commandArray[i] = Commands.defer(() -> swerve.pathfindthenFollowPath(position),Set.of(swerve,operatorControl));
			}
			Command ret = new SequentialCommandGroup(commandArray);
			return ret;
		} else {
			return autoChooser.getSelected().construct();
		}
	}

	public static double getAutonomousDelay() {
		return autoDelayChooser.getSelected();
	}

}