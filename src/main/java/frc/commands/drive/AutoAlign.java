package frc.commands.drive;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.PhotonvisionConstants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import java.util.List;

public class AutoAlign extends Command {

  double angle;
  PIDController controller;
  AprilTagFieldLayout tagLayout;
  int tag;


  private double normalizeAngle(double angle) {
      angle %= 360;

      if (angle >= 180) {
        angle -= 360;
      }
  
      if (angle <= -180) {
        angle += 360;
      }
  
      return angle;
  }

  public AutoAlign(int tag) {
    this.tag = tag;
    addRequirements(Robot.swerve);


    controller = new PIDController(0.07, 0, 0.01);
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(2);
  }

  @Override
  public void initialize() {
    this.angle = Robot.swerve.calculateFieldPosition(tag).getRotation().getDegrees();
  }

  @Override
  public void execute() {

    System.out.println(angle);
    Robot.swerve.drive(new Translation2d(), controller.calculate(normalizeAngle(Robot.navX.getAngle()), angle), true);
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();// || Math.abs(normalizeAngle(Robot.navX.getAngle()) - angle) < 2;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.drive(new Translation2d(), 0, true);

  }
}
