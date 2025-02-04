
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;



import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.autonomous.Autonomous;
import frc.commands.drive.AutoAlign;
import frc.autonomous.AutonomousProgram;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.commands.drive.DefaultDrive;
import frc.commands.drive.DriveWait;
import frc.commands.drive.TurnToAngle;
import frc.modules.OperatorControlModule;
import frc.modules.PhotonvisionModule;
import frc.robot.util.LocalADStarAK;
// import frc.commands.drive.TurnToSpeaker;
import frc.subsystems.DriveSubsystem;


public class Robot extends LoggedRobot {
	public static final CommandXboxController driverController = new CommandXboxController(0);
	public static final CommandXboxController operatorController = new CommandXboxController(1);
	public static final Joystick leftJoystick = new Joystick(2);
	public static final Joystick rightJoystick = new Joystick(3);



	public static final AHRS navX = new AHRS();
	public static PhotonvisionModule photonvision;
	public static DriveSubsystem swerve;
	public static OperatorControlModule operatorControl;
	// public static Logger logger;

	public static enum FieldPosition {
		RED_REEF_A,
		RED_REEF_B,
		RED_REEF_C,
		RED_REEF_D,
		RED_REEF_E,
		RED_REEF_F,
		RED_REEF_G,
		RED_REEF_H,
		RED_REEF_I,
		RED_REEF_J,
		RED_REEF_K,
		RED_REEF_L,
		BLUE_REEF_A,
		BLUE_REEF_B,
		BLUE_REEF_C,
		BLUE_REEF_D,
		BLUE_REEF_E,
		BLUE_REEF_F,
		BLUE_REEF_G,
		BLUE_REEF_H,
		BLUE_REEF_I,
		BLUE_REEF_J,
		BLUE_REEF_K,
		BLUE_REEF_L,
		BLUE_LEFT_CORALSUBSTATION,
		BLUE_RIGHT_CORALSUBSTATION,
		RED_LEFT_CORALSUBSTATION,
		RED_RIGHT_CORALSUBSTATION,
		BLUE_PROCESSOR,
		RED_PROCESSOR,
		RED_BARGE,
		BLUE_BARGE,
		CURRENT_POSE
	}
	public static enum ReefPosition {
		RED_REEF_A,
		RED_REEF_B,
		RED_REEF_C,
		RED_REEF_D,
		RED_REEF_E,
		RED_REEF_F,
		RED_REEF_G,
		RED_REEF_H,
		RED_REEF_I,
		RED_REEF_J,
		RED_REEF_K,
		RED_REEF_L,
		BLUE_REEF_A,
		BLUE_REEF_B,
		BLUE_REEF_C,
		BLUE_REEF_D,
		BLUE_REEF_E,
		BLUE_REEF_F,
		BLUE_REEF_G,
		BLUE_REEF_H,
		BLUE_REEF_I,
		BLUE_REEF_J,
		BLUE_REEF_K,
		BLUE_REEF_L,
	}

	@Override
	public void robotInit() {
		photonvision = new PhotonvisionModule();
		swerve = new DriveSubsystem();
		// swerve = new SSubsystem();
		operatorControl = new OperatorControlModule();
		//Subsystem initialization goes here
		swerve.resetOdometry();
		NamedCommands.registerCommand("Auto Align 9", new SequentialCommandGroup(
			new AutoAlign(FieldPosition.RED_REEF_E).withTimeout(6)
		));

		NamedCommands.registerCommand("Auto Align 11", new SequentialCommandGroup(
			new AutoAlign(FieldPosition.RED_REEF_I).withTimeout(6)
		));

		//NamedCommands.registerCommands() goes here
		configureButtonBindings();

		Logger.recordMetadata("Java-Command-2024", "robot"); // Set a metadata value

		if (isReal()) {
			//Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			new PowerDistribution(13, ModuleType.kRev); // Enables power distribution logging
		} else {
			// setUseTiming(false); // Run as fast as possible
			// String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
			// // AdvantageScope (or prompt the
			// // // user)
			// //
			// Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			// Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,"_sim"))); // Save outputs to a
			Logger.addDataReceiver(new NT4Publisher()); // new log
		}

		Logger.start();

	


		Autonomous.init();
		AutonomousProgram.addAutosToShuffleboard();
		Pathfinding.setPathfinder(new LocalADStarAK());
		//setDefaultCommand initialization goes here
		swerve.setDefaultCommand(new DefaultDrive(true));
		swerve.resetOnlyNavX();
		CameraServer.startAutomaticCapture();
		//Driver Camera code:
		// CvSink cvSink = CameraServer.getVideo();
		// CvSource outputStream = CameraServer.putVideo("Blur",1280,720);

	}

	@Override
	public void disabledInit() {
		swerve.periodicReset();
	}

	@Override
	public void disabledPeriodic() {
		operatorControl.searchChange();
	}

	// @Override
	// public void simulationInit() {
	// 	configureButtonBindings();
	// }

	private void configureButtonBindings() {
		// driverController.y().onTrue(Commands.runOnce(() ->
		// swerve.resetOdometry(swerve.getPose())));
		
		// driverController.y().onTrue(new InstantCommand(swerve::resetOnlyNavX));
		driverController.y().onTrue(Commands.runOnce(() -> swerve.resetOdometry()));
		operatorController.pov(0).onTrue(Commands.runOnce(() -> operatorControl.moveUp()));
		operatorController.pov(90).onTrue(Commands.runOnce(() -> operatorControl.moveRight()));
		operatorController.pov(180).onTrue(Commands.runOnce(() -> operatorControl.moveDown()));
		operatorController.pov(270).onTrue(Commands.runOnce(() -> operatorControl.moveLeft()));
		operatorController.start().onTrue(Commands.runOnce(()-> operatorControl.queuePlacement()));
		operatorController.back().onTrue(Commands.runOnce(()-> operatorControl.setPiece()));
		operatorController.rightBumper().onTrue(Commands.runOnce(() -> operatorControl.toggleStrategy()));





		// if(DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get()==Alliance.Blue){
		// 	driverController.leftBumper().onTrue(swerve.pathfindthenFollowPath(FieldPosition.BLUE_LEFT_CORALSUBSTATION));
		// } else {
		// 	driverController.leftBumper().onTrue(swerve.pathfindthenFollowPath(FieldPosition.RED_LEFT_CORALSUBSTATION));
		// }
		// if(DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get()==Alliance.Blue){
		// 	driverController.rightBumper().onTrue(swerve.pathfindthenFollowPath(FieldPosition.BLUE_RIGHT_CORALSUBSTATION));
		// } else {
		// 	driverController.rightBumper().onTrue(swerve.pathfindthenFollowPath(FieldPosition.RED_RIGHT_CORALSUBSTATION));
		// }
		//TODO: Add scoring routine into start button
		//TODO: Add scoring routine into start button
		//TODO: Add scoring routine into start button
		//TODO: Add scoring routine into start button
		// driverController.start().onTrue(Commands.runOnce(() -> operatorControl.lockIn()).andThen(swerve.pathfindthenFollowPath(FieldPosition.RED_REEF_A), Commands.runOnce(() -> operatorControl.lockOut())));
		driverController.start().toggleOnTrue(new SequentialCommandGroup(Commands.runOnce(() -> operatorControl.lockIn()),Commands.runOnce(()->swerve.pathfindthenFollowPath(operatorControl.getQueuedPosition()).addRequirements(swerve)),Commands.runOnce(() -> operatorControl.lockOut())));
		// driverController.start().toggleOnTrue(new SequentialCommandGroup(Commands.runOnce(() -> System.out.println("1")),Commands.runOnce(() -> System.out.println("2")),Commands.runOnce(() -> System.out.println("3"))));
			// Commands.runOnce(() -> operatorControl.lockIn()).andThen(swerve.pathfindthenFollowPath(FieldPosition.RED_REEF_A), Commands.runOnce(() -> operatorControl.lockOut())));
		// driverController.start().onTrue((Commands.runOnce(() -> operatorControl.lockIn()).andThen(() -> System.out.println("woah")).andThen(() -> operatorControl.lockOut())).unless(() -> operatorControl.queuedValue == null));

		// driverController.a().toggleOnTrue(new TurnToAngle(0).repeatedly());

		driverController.a().onTrue(new AutoAlign(FieldPosition.RED_REEF_E));
		// driverController.x().onTrue(swerve.pathfindthenFollowPath(FieldPosition.RED_REEF_I));
		// driverController.b().onTrue(swerve.pathfindthenFollowPath(FieldPosition.RED_REEF_E));

		//Game-specific Button Bindings go here


	}

	/* Currently running auto routine */

	private Command autonomousCommand;

	@Override
	public void autonomousInit() {
		navX.reset();
		Command autonomousRoutine = AutonomousProgram.constructSelectedRoutine();

		// Home the arm while waiting for the drivebase delay
		Command delay = new ParallelCommandGroup(new DriveWait(AutonomousProgram.getAutonomousDelay()));

		// Schedule the selected autonomous command group
		if (autonomousRoutine != null) {
			// Run the delay/home and the selected routine sequentially
			this.autonomousCommand = new SequentialCommandGroup(
					delay,
					autonomousRoutine);
		} else {
			this.autonomousCommand = delay;
		}

		// Schedule the combined command group
		CommandScheduler.getInstance().schedule(this.autonomousCommand);
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

	}


}