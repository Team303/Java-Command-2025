package frc.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;


import static frc.autonomous.AutonomousProgram.create;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * Quick guide to Comand Groups:
 *
 * SequentialComandGroup:
 * Will run all comands in order within it's parentheses
 * Note: If a comand does not have a isFinshed statment the code will be stuck
 * on that command foreverk
 *
 * ParallelCommandGroup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Both commands will have to finish to move on
 *
 * ParallelRaceGoup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: As soon as one command runs it's isfinished method runs then both
 * commands will end
 *
 * ParallelDeadlineGroup
 * Will run commands in parallel if they use diffrent    SubSystems
 * Note: Only the first command will finish the group
 */
public class Autonomous {


    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    // global event map

    // This is just an example event map. It would be better to have a constant,
    // in your code that will be used by all path following commands.

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.

    public static void init() {
        double xPos = Units.inchesToMeters(150.49);
        double yPos = Units.inchesToMeters(100.17);
        
        create("TEST", () -> new PathPlannerAuto("Test"));
    }
public static SequentialCommandGroup setAngleAdjustmentStart(double angleDeg, double xPos, double yPos, String commandName) {
        Robot.navX.reset();
        Robot.swerve.resetPose(new Pose2d(new Translation2d(xPos, yPos), new Rotation2d(angleDeg)));
        return new SequentialCommandGroup(new InstantCommand(() -> Robot.navX.setAngleAdjustment(angleDeg)), Robot.swerve.getAutonomousCommand(commandName));
    }
    
}