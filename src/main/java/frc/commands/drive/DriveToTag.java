package frc.commands.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.subsystems.DriveSubsystem;

public class DriveToTag extends Command {
    int tag;

    // boolean isAlliance = true;

    public DriveToTag(int tag) {
        addRequirements(Robot.swerve);
        this.tag=tag;
    }

    @Override
    public void execute() {
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d(new Translation2d(Units.inchesToMeters(150.49), Units.inchesToMeters(100.17)),
            Rotation2d.fromDegrees(0)));
        PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(Robot.swerve.calculateAngleFieldPosition(tag))) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
);
        path.preventFlipping=true;
        AutoBuilder.followPath(path);
    }
}