// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.modules.PhotonvisionModule.CameraName;
import frc.modules.SwerveModule;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Robot.FieldPosition;
import frc.robot.RobotMap.PhotonvisionConstants;
import frc.robot.RobotMap.Swerve;
import edu.wpi.first.wpilibj.Alert;

import static frc.robot.Robot.operatorControl;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {

  private boolean speakerLock = false;
  private boolean ampLock = false;
  public static final double kMaxSpeed = 5.2 * 4; // 5.2 meters per second
  public static final double kMaxAngularSpeed = kMaxSpeed / (Math.hypot(0.381, 0.381)); // radians per second

  private final Translation2d frontLeftLocation = new Translation2d(Units.inchesToMeters(24.5 / 2.0),
      Units.inchesToMeters(24.0));
  private final Translation2d frontRightLocation = new Translation2d(Units.inchesToMeters(24.5 / 2.0),
      -Units.inchesToMeters(24.0));
  private final Translation2d backLeftLocation = new Translation2d(-Units.inchesToMeters(24.5 / 2.0),
      Units.inchesToMeters(24.0));
  private final Translation2d backRightLocation = new Translation2d(-Units.inchesToMeters(24.5 / 2.0),
      -Units.inchesToMeters(24.0));

  public final SwerveModule frontLeft;
  public final SwerveModule frontRight;
  public final SwerveModule backLeft;
  public final SwerveModule backRight;
  // private final SwerveDriveOdometry odometry;
  private final PIDController driftCorrectionPid = new PIDController(0.12, 0, 0);
  private final PIDController speakerAlignPid = new PIDController(0.5, 0, 0);
  // private Pose2d pose = new Pose2d(0.0, 0.0, new Rotation2d());

  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  private ChassisSpeeds relativeSpeeds = new ChassisSpeeds();

  public AprilTagFieldLayout initialLayout;

  public static final ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Drive Base");

  public static final GenericEntry FRONT_LEFT_ENC = DRIVEBASE_TAB.add("front left enc", 0).withPosition(0, 0)
      .withSize(2, 1).getEntry();
  public static final GenericEntry FRONT_RIGHT_ENC = DRIVEBASE_TAB.add("front right enc", 0).withPosition(2, 0)
      .withSize(2, 1).getEntry();
  public static final GenericEntry BACK_LEFT_ENC = DRIVEBASE_TAB.add("back left enc", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();
  public static final GenericEntry BACK_RIGHT_ENC = DRIVEBASE_TAB.add("back right enc", 0).withPosition(7, 0)
      .withSize(2, 1).getEntry();

  public static final GenericEntry backRightDriveEncoder = DRIVEBASE_TAB.add("back right drive enc", 0)
      .withPosition(7, 1).withSize(2, 1).getEntry();
  public static final GenericEntry backLeftDriveEncoder = DRIVEBASE_TAB.add("back left drive enc", 0)
      .withPosition(5, 1).withSize(2, 1).getEntry();
  public static final GenericEntry frontLeftDriveEncoder = DRIVEBASE_TAB.add("front left drive enc", 0)
      .withPosition(0, 1).withSize(2, 1).getEntry();
  public static final GenericEntry frontRightDriveEncoder = DRIVEBASE_TAB.add("front right drive enc", 0)
      .withPosition(2, 1).withSize(2, 1).getEntry();

  public static final GenericEntry backRightTurnOutput = DRIVEBASE_TAB.add("back right turn ouptut", 0)
      .withPosition(7, 2).withSize(2, 1).getEntry();
  public static final GenericEntry backLeftTurnOutput = DRIVEBASE_TAB.add("back left turn ouptut", 0).withPosition(5, 2)
      .withSize(2, 1).getEntry();
  public static final GenericEntry frontLeftTurnOutput = DRIVEBASE_TAB.add("front left turn ouptut", 0)
      .withPosition(0, 2).withSize(2, 1).getEntry();
  public static final GenericEntry frontRightTurnOutput = DRIVEBASE_TAB.add("front right turn ouptut", 0)
      .withPosition(2, 2).withSize(2, 1).getEntry();

  public static final GenericEntry backRightAngle = DRIVEBASE_TAB.add("back right angle", 0).withPosition(7, 3)
      .withSize(2, 1).getEntry();
  public static final GenericEntry backLeftAngle = DRIVEBASE_TAB.add("back left angle", 0).withPosition(5, 3)
      .withSize(2, 1).getEntry();
  public static final GenericEntry frontLeftAngle = DRIVEBASE_TAB.add("front left angle", 0).withPosition(0, 3)
      .withSize(2, 1).getEntry();
  public static final GenericEntry frontRightAngle = DRIVEBASE_TAB.add("front right angle", 0).withPosition(2, 3)
      .withSize(2, 1).getEntry();

  public static final GenericEntry resetPoseX = DRIVEBASE_TAB.add("resetPoseX", 0).withPosition(0, 4)
      .withSize(1, 1).getEntry();
  public static final GenericEntry resetPoseY = DRIVEBASE_TAB.add("resetPoseY", 0).withPosition(1, 4)
      .withSize(1, 1).getEntry();
  public static final GenericEntry resetPoseAngle = DRIVEBASE_TAB.add("resetPoseAngle", 0).withPosition(2, 4)
      .withSize(1, 1).getEntry();

  public static final GenericEntry globalAngle = DRIVEBASE_TAB.add("global angle", 0).withPosition(4, 0).getEntry();
  public static final GenericEntry angleVelo = DRIVEBASE_TAB.add("angular velocity", 0).withPosition(4, 1).getEntry();
  // public static final GenericEntry time = DRIVEBASE_TAB.add("Time",
  // 0).withPosition(4, 2).getEntry();

  public static final GenericEntry translationalVelo = DRIVEBASE_TAB.add("transational velocity", 0).withPosition(4, 3)
      .getEntry();

  public static final GenericEntry angleToSpeaker = DRIVEBASE_TAB.add("angleToSpeaker", 0).withPosition(3, 4)
      .getEntry();

  private final Alert logFrontLeftVisionJittery = new Alert("Operator Terminal",
      "Front left vision jittery, pose not taken", AlertType.kWarning);
  private final Alert logFrontRightVisionJittery = new Alert("Operator Terminal",
      "Front right vision jittery, pose not taken", AlertType.kWarning);
  private final Alert logFrontLeftOOB = new Alert("Operator Terminal",
      "Front left vision out of bounds, pose not taken", AlertType.kWarning);
  private final Alert logFrontRightOOB = new Alert("Operator Terminal",
      "Front right vision out of bounds, pose not taken", AlertType.kWarning);
  private final Alert logNoNodeSelected = new Alert("Operator Terminal","No field position sleected, path not run",AlertType.kError);

  public static double angularVelocity = 0;

  private double m_desiredHeading = 0;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final Timer AVTimer = new Timer();

  public final AprilTagFieldLayout aprilTagField;
  private final Field2d field2d = new Field2d();
  private static final Vector<N3> odometryStandardDeviations = VecBuilder.fill(5, 5, Units.degreesToRadians(10));
  // private static final Vector<N3> odometryStandardDeviations =
  // VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE);
  private static final Vector<N3> photonStandardDeviations = VecBuilder.fill(5, 5, Integer.MAX_VALUE - 101);
  private static final Vector<N3> kSingleStandardDeviations = VecBuilder.fill(5, 5, Integer.MAX_VALUE - 101);
  private static final Vector<N3> kMultiTagStandardDeviations = VecBuilder.fill(2.5, 2.5, Integer.MAX_VALUE - 101);

  public PhotonPoseEstimator[] visionPoseEstimator = new PhotonPoseEstimator[2];

  public CANcoderConfiguration configLeftFront;
  public CANcoderConfiguration configRightFront;
  public CANcoderConfiguration configLeftBack;
  public CANcoderConfiguration configRightBack;

  public SwerveDrivePoseEstimator poseEstimator;

  EstimatedRobotPose frontLeftVisionPose;
  EstimatedRobotPose frontRightVisionPose;

  public DriveSubsystem() {

    Logger.recordOutput("Swerve Module States", new SwerveModuleState[] { new SwerveModuleState(),
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() });
    Robot.navX.reset();
    AVTimer.start();

    configLeftFront = new CANcoderConfiguration();
    configLeftFront.MagnetSensor.MagnetOffset = Swerve.LEFT_FRONT_STEER_OFFSET;
    configRightFront = new CANcoderConfiguration();
    configRightFront.MagnetSensor.MagnetOffset = Swerve.RIGHT_FRONT_STEER_OFFSET;
    configLeftBack = new CANcoderConfiguration();
    configLeftBack.MagnetSensor.MagnetOffset = Swerve.LEFT_BACK_STEER_OFFSET;
    configRightBack = new CANcoderConfiguration();
    configRightBack.MagnetSensor.MagnetOffset = Swerve.RIGHT_BACK_STEER_OFFSET;

    frontLeft = new SwerveModule("Left Front",
        RobotMap.Swerve.LEFT_FRONT_DRIVE_ID,
        RobotMap.Swerve.LEFT_FRONT_STEER_ID,
        "rio",
        "rio",
        RobotMap.Swerve.LEFT_FRONT_STEER_CANCODER_ID,
        0.2,
        0.0,
        0.0,
        50,
        0.0,
        0.0,
        configLeftFront);

    frontRight = new SwerveModule("Right Front",
        RobotMap.Swerve.RIGHT_FRONT_DRIVE_ID,
        RobotMap.Swerve.RIGHT_FRONT_STEER_ID,
        "rio",
        "rio",
        RobotMap.Swerve.RIGHT_FRONT_STEER_CANCODER_ID,
        0.2,
        0.0,
        0.0,
        50,
        0.0,
        0.0,
        configRightFront);
    backLeft = new SwerveModule("Left Back",
        RobotMap.Swerve.LEFT_BACK_DRIVE_ID,
        RobotMap.Swerve.LEFT_BACK_STEER_ID,
        "rio",
        "rio",
        RobotMap.Swerve.LEFT_BACK_STEER_CANCODER_ID,
        0.2,
        0.0,
        0.0,
        50,
        0.0,
        0.0,
        configLeftBack);
    backRight = new SwerveModule("Right Back",
        RobotMap.Swerve.RIGHT_BACK_DRIVE_ID,
        RobotMap.Swerve.RIGHT_BACK_STEER_ID,
        "rio",
        "rio",
        RobotMap.Swerve.RIGHT_BACK_STEER_CANCODER_ID,
        0.2,
        0.0,
        0.0,
        50,
        0.0,
        0.0,
        configRightBack);

    // frontLeft.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.LEFT_FRONT_STEER_OFFSET);
    // frontRight.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.RIGHT_FRONT_STEER_OFFSET);
    // backLeft.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.LEFT_BACK_STEER_OFFSET);
    // backRight.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.RIGHT_BACK_STEER_OFFSET);

    frontLeft.invertSteerMotor(true);
    frontRight.invertSteerMotor(true);
    backRight.invertSteerMotor(true);
    backLeft.invertSteerMotor(true);

    frontLeft.invertDriveMotor(false);
    backLeft.invertDriveMotor(true);
    frontRight.invertDriveMotor(false);
    backRight.invertDriveMotor(false);

    // frontLeft.getDrivePosition();
    // frontRight.getDrivePosition();
    // backLeft.getDrivePosition();
    // backRight.getDrivePosition();

    try {
      initialLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      Optional<Alliance> alliance = DriverStation.getAlliance();
      // TODO: Change to make the origin position based off of station rather than
      // TODO: Make set origin to Blue no matter what for pathplanner compatibility
      // just based off of alliance.
      initialLayout
          .setOrigin(alliance.isPresent() && alliance.get() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
              : OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      initialLayout = null;
    }
    aprilTagField = initialLayout;
    // if (Robot.isReal()) {

    visionPoseEstimator[0] = new PhotonPoseEstimator(aprilTagField,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonvisionConstants.ROBOT_TO_FRONT_LEFT_CAMERA);
    // // visionPoseEstimatorRight = new PhotonPoseEstimator(aprilTagField,
    // // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    // // Robot.photonvision.getCamera(CameraName.CAM2),
    // // PhotonvisionConstants.ROBOT_TO_RIGHT_CAMERA);
    visionPoseEstimator[1] = new PhotonPoseEstimator(aprilTagField,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonvisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA);
    // // visionPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagField,
    // // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    // // Robot.photonvision.getCamera(CameraName.CAM4),
    // // PhotonvisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA);
    // visionPoseEstimator[0].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // //
    // visionPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // visionPoseEstimator[1].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // //
    // visionPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // }

    poseEstimator = new SwerveDrivePoseEstimator(kinematics, Robot.navX.getRotation2d(), getModulePositions(),
        new Pose2d(new Translation2d(), new Rotation2d()),
        odometryStandardDeviations, photonStandardDeviations);

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController(new PIDConstants(5, 0.0, 0.0), new PIDConstants(8, 0, 0)), new RobotConfig(
            Units.lbsToKilograms(RobotMap.Swerve.ROBOT_MASS),
            RobotMap.Swerve.ROBOT_MOI,
            new ModuleConfig(Units.inchesToMeters(2), kMaxSpeed, kMaxAngularSpeed,
                new DCMotor(12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000), 1), 130, 1),
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  // public void followTrajectory(SwerveSample sample) {
  // // Get the current pose of the robot
  // Pose2d pose = getPose();

  // // Generate the next speeds for the robot
  // ChassisSpeeds speeds = new ChassisSpeeds(
  // sample.vx + xController.calculate(pose.getX(), sample.x),
  // sample.vy + yController.calculate(pose.getY(), sample.y),
  // sample.omega + headingController.calculate(pose.getRotation().getRadians(),
  // sample.heading)
  // );

  // // Apply the generated speeds
  // drive();
  // }

  /**
   * Adds rotational velocity to the chassis speed to compensate for
   * unwanted changes in gyroscope heading.
   * 
   * @param chassisSpeeds the given chassisspeeds
   * @return the corrected chassisspeeds
   */
  private ChassisSpeeds translationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
    if (!Robot.navX.isConnected())
      return chassisSpeeds;
    double translationalVelocity = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Logger.recordOutput("translational velocity", translationalVelocity);
    Logger.recordOutput("turn rate", Robot.navX.getRate());
    if (Math.abs(Robot.navX.getRate()) > 0.5) {
      m_desiredHeading = Robot.navX.getAngle();
    } else if (Math.abs(translationalVelocity) > 1) {

      double calc = driftCorrectionPid.calculate(Robot.navX.getAngle(),
          m_desiredHeading);

      if (Math.abs(calc) >= 0.1) {
        chassisSpeeds.omegaRadiansPerSecond -= calc;
      }
    }
    return chassisSpeeds;
  }

  public void drive(SwerveModuleState[] state) {

    // frontLeftAngle.setDouble(state[0].angle.getDegrees());
    // frontRightAngle.setDouble(state[1].angle.getDegrees());
    backLeftAngle.setDouble(state[2].angle.getDegrees());
    // backRightAngle.setDouble(state[3].angle.getDegrees());

    Logger.recordOutput("Swerve Module States", state);

    frontLeft.setDesiredState(state[0]);
    frontRight.setDesiredState(state[1]);
    backLeft.setDesiredState(state[2]);
    backRight.setDesiredState(state[3]);
  }

  private double normalizeAngle(double angleDeg) {
    angleDeg %= 360;
    if (Math.abs(angleDeg) < 180)
      return angleDeg;
    else if (angleDeg > 0)
      return angleDeg - 360;
    else
      return angleDeg + 360;
  }

  public ChassisSpeeds ampAlign(ChassisSpeeds chassisSpeeds) {

    boolean isBlue = true;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isBlue = alliance.get() == DriverStation.Alliance.Blue;
    }

    double angle = isBlue ? -90 : 90;

    chassisSpeeds.omegaRadiansPerSecond -= speakerAlignPid.calculate(Robot.navX.getAngle(), angle);

    return chassisSpeeds;
  }

  public void setSpeakerLock() {
    speakerLock = true;
  }

  public void setAmpLock() {
    ampLock = true;
  }

  public void removeLock() {
    ampLock = false;
    speakerLock = false;
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {

    ChassisSpeeds chassisSpeeds = fieldOriented
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
            Rotation2d.fromDegrees(Robot.navX.getAngle()))
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    // chassisSpeeds = translationalDriftCorrection(chassisSpeeds);

    // lock onto different field elements (methods will change the anglular
    // velocity)

    var swerveModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 0.02));

    Logger.recordOutput("Swerve Module States", swerveModuleStates);

    drive(swerveModuleStates);
  }

  public void robotRelativeDrive(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedForwards) {

    drive(kinematics.toSwerveModuleStates(chassisSpeeds));

  }

  public PathPlannerPath pathFromFile(String pathToFile) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathToFile);
      return path;
    } catch (Exception e) {
      System.out.println(e);
      return null;
    }
  }

  public Command getAutonomousCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }

  public Command followPathOnTheFly(FieldPosition position) {
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this
                                                                                           // path.
    List<Waypoint> waypoints = PathPlannerPath
        .waypointsFromPoses(Robot.swerve.getPose(), new Pose2d(calculateFieldPosition(position).getTranslation(),
            Rotation2d.fromDegrees(Robot.swerve.calculateFieldPosition(position).getRotation().getDegrees() + 60)));
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be null for on-the-fly paths.
        new GoalEndState(0.0,
            Rotation2d.fromDegrees(Robot.swerve.calculateFieldPosition(position).getRotation().getDegrees() + 60)) // Goal
    // end
    // state.
    // You can set a
    // holonomic
    // rotation here.
    // If using a
    // differential
    // drivetrain, the
    // rotation will
    // have no effect.
    );
    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }

  public Command followPathfinding(FieldPosition position) {
    Pose2d targetPose = new Pose2d(calculateFieldPosition(position).getTranslation(),
        Rotation2d.fromDegrees(Robot.swerve.calculateFieldPosition(position).getRotation().getDegrees() + 60));
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  public Command pathfindthenFollowPath(FieldPosition position) {
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this
    if (position.name().equals("CURRENT_POSE")) {
      logNoNodeSelected.set(true);
      return Commands.none();
    } else {
      logNoNodeSelected.set(false);
      return AutoBuilder.pathfindThenFollowPath(pathFromFile(position.name()), constraints);
    }
  }

  // /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    Optional<EstimatedRobotPose> resultFrontLeft = getEstimatedGlobalPose(poseEstimator.getEstimatedPosition(),
        CameraName.FRONT_LEFT);
    Optional<EstimatedRobotPose> resultFrontRight = getEstimatedGlobalPose(poseEstimator.getEstimatedPosition(),
        CameraName.FRONT_RIGHT);

    // int cameraCount=0;
    if (resultFrontLeft.isPresent()) {
      EstimatedRobotPose visionPoseEstimate = resultFrontLeft.get();
      Pose3d estPose = visionPoseEstimate.estimatedPose;
      if (poseInBounds(estPose.getX(), estPose.getY())) {
        logFrontLeftOOB.set(false);
        if (!visionJittery(frontLeftVisionPose, visionPoseEstimate)) {
          logFrontLeftVisionJittery.set(false);
          frontLeftVisionPose=visionPoseEstimate;
          Vector<N3> stddevs = getEstimationStdDevs(visionPoseEstimate.targetsUsed);
          // double[] data = stddevs.getData();
          poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(),
              visionPoseEstimate.timestampSeconds,
              stddevs);
        } else {
          logFrontLeftVisionJittery.set(true);
        }
      } else {
        logFrontLeftOOB.set(true);
      }
      // cameraCount++;
    }
    if (resultFrontRight.isPresent()) {
      EstimatedRobotPose visionPoseEstimate = resultFrontRight.get();
      Pose3d estPose = visionPoseEstimate.estimatedPose;
      if (poseInBounds(estPose.getX(), estPose.getY())) {
        logFrontRightOOB.set(false);
        if (!visionJittery(frontRightVisionPose, visionPoseEstimate)) {
          logFrontRightVisionJittery.set(false);
          frontRightVisionPose=visionPoseEstimate;
          Vector<N3> stddevs = getEstimationStdDevs(visionPoseEstimate.targetsUsed);
          // double[] data = stddevs.getData();
          poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(),
              visionPoseEstimate.timestampSeconds,
              stddevs);
        } else {
          logFrontRightVisionJittery.set(true);
        }
      } else {
        logFrontRightOOB.set(true);
      }
      // cameraCount++;
    }

    poseEstimator.update(Robot.navX.getRotation2d().times(-1), getModulePositions());
    field2d.setRobotPose(getPose());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public boolean poseInBounds(double x, double y) {
    return x > 0.0 && x < RobotMap.FieldConstants.fieldLength && y > 0.0 && y < RobotMap.FieldConstants.fieldWidth;
  }

  public boolean visionJittery(EstimatedRobotPose prevPose, EstimatedRobotPose curPose) {
    if (prevPose == null) {
      return false;
    } else if (curPose.estimatedPose.getTranslation().getDistance(
        prevPose.estimatedPose.getTranslation()) < Math.abs(curPose.timestampSeconds - prevPose.timestampSeconds)
            * Math.hypot(Robot.navX.getVelocityY(), Robot.navX.getVelocityX()) * RobotMap.Swerve.JITTER_FACTOR) {
      return false;
    } else {
      return true;
    }
  }

  public Vector<N3> getEstimationStdDevs(List<PhotonTrackedTarget> targetList) {
    var estStdDevs = kSingleStandardDeviations;
    var targets = targetList;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = visionPoseEstimator[0].getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(getPose().getTranslation());
    }
    if (numTags == 0)
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    ;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = kMultiTagStandardDeviations;
    // Increase std devs based on (average) distance
    // if (numTags == 1 && avgDist > 4)
    // estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
    // Double.MAX_VALUE);
    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist *
        RobotMap.Swerve.PHOTON_STDDEV_SCALING_FACTOR));

    return estStdDevs;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, CameraName camera) {
    int estimator;
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    switch (camera) {
      case FRONT_LEFT:
        estimator = 0;
        break;
      case FRONT_RIGHT:
        estimator = 1;
        break;
      default:
        System.out.println("DriveSubsystem get global pose is accessing illegal camera.");
        return Optional.empty();
    }
    visionPoseEstimator[estimator].setReferencePose(prevEstimatedRobotPose);
    if (Robot.photonvision.hasTargets(camera)) {
      PhotonCamera photonCamera = Robot.photonvision.getCamera(camera);
      List<PhotonPipelineResult> rawResults = photonCamera.getAllUnreadResults();
      for (int i = 0; i < rawResults.size(); i++) {
        List<PhotonTrackedTarget> targets = rawResults.get(i).targets;
        for (int j = 0; j < targets.size(); j++) {
          if (targets.get(j).getPoseAmbiguity() > 0.25) {
            targets.remove(j);
            --j;
          }
        }
        visionEst = visionPoseEstimator[estimator]
            .update(new PhotonPipelineResult(rawResults.get(i).metadata, targets, Optional.empty()));
        // TODO: Check if this is necessary
        // long sequenceID = rawResults.get(i).metadata.sequenceID;
      }
      return visionEst;
    } else {
      return Optional.empty();
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry() {

    boolean isBlue = true;

    // depending on which alliance, set which global direction to zero to (0 or 180)
    // BLUE ORIGINg

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isBlue = alliance.get() == DriverStation.Alliance.Blue;
    }

    Robot.navX.reset();

    poseEstimator.resetPosition(Robot.navX.getRotation2d(), getModulePositions(),
        new Pose2d(
            new Translation2d(calculateFieldPosition(FieldPosition.RED_REEF_C).getX(),
                calculateFieldPosition(FieldPosition.RED_REEF_C).getY()),
            Rotation2d.fromDegrees(isBlue ? 0 : 60)));

  }

  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX
  // RESET ODOMETRY FIX

  public void resetOnlyNavX() {
    Robot.navX.reset();
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometryWidget() {

    // resets pose based on values inputted on shuffleboard
    // depending on which alliance, set which global direction to zero (0 or 180)
    // BLUE ORIGIN

    boolean isBlue = true;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isBlue = alliance.get() == DriverStation.Alliance.Blue;
    }

    Robot.navX.reset();

    resetOdometry(new Pose2d(
        new Translation2d(
            resetPoseX.getDouble(0),
            resetPoseY.getDouble(0)),
        Rotation2d.fromDegrees(isBlue ? 0 : 180)));
  }

  public Pose2d calculateFieldPosition(FieldPosition position) {

    switch (position) {
      // TODO: Fix angles to be relative to real starting position
      // TODO: Add support for blue alliance
      case RED_LEFT_CORALSUBSTATION:
        return new Pose2d(1, 1, Rotation2d.fromDegrees(60.0));
      case RED_RIGHT_CORALSUBSTATION:
        return new Pose2d(1, 1, Rotation2d.fromDegrees(180.0));
      case RED_PROCESSOR:
        return new Pose2d(1, 1, Rotation2d.fromDegrees(210.0));
      case RED_BARGE:
        return new Pose2d(1, 1, Rotation2d.fromDegrees(300.0));
      case RED_REEF_K:
        return new Pose2d(1, 1, Rotation2d.fromDegrees(240.0));
      case RED_REEF_L:
        return new Pose2d(1, 1, Rotation2d.fromDegrees(240.0));
      case RED_REEF_A:
        return new Pose2d(2.9, 4.025, Rotation2d.fromDegrees(300.0));
      case RED_REEF_B:
        return new Pose2d(2.9, 4.025, Rotation2d.fromDegrees(300.0));
      case RED_REEF_C:
        return new Pose2d(3.82, 2.54, Rotation2d.fromDegrees(0.0));
      case RED_REEF_D:
        return new Pose2d(3.82, 2.54, Rotation2d.fromDegrees(0.0));
      case RED_REEF_E:
        return new Pose2d(5.29, 2.62, Rotation2d.fromDegrees(60.0));
      case RED_REEF_F:
        return new Pose2d(5.29, 2.62, Rotation2d.fromDegrees(60.0));
      case RED_REEF_G:
        return new Pose2d(5.95, 4.12, Rotation2d.fromDegrees(120.0));
      case RED_REEF_H:
        return new Pose2d(5.95, 4.12, Rotation2d.fromDegrees(120.0));
      case RED_REEF_I:
        return new Pose2d(5.33, 5.33, Rotation2d.fromDegrees(180.0));
      case RED_REEF_J:
        return new Pose2d(5.33, 5.33, Rotation2d.fromDegrees(180.0));
      case CURRENT_POSE:
        return getPose();
      default:
        System.out.println("calculateFieldPosition() received an invalid position");
        return getPose();
    }
    // boolean isBlue = true;

    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    // isBlue = alliance.get() == DriverStation.Alliance.Blue;
    // }

    // Pose2d robotPose = getPose();
    // Translation2d speakerPose;

    // speakerPose = isBlue ?
    // RobotMap.FieldConstants.centerSpeakOpenInBlue.getTranslation()
    // : RobotMap.FieldConstants.centerSpeakOpenInRed.getTranslation();

    // return Math.atan2(speakerPose.getY() - robotPose.getY(), speakerPose.getX() -
    // robotPose.getX()) * (180 / Math.PI)
    // * (isBlue ? -1 : 0);

  }

  public void periodicReset() {
    frontLeft.periodicReset();
    frontRight.periodicReset();
    backLeft.periodicReset();
    backRight.periodicReset();
  }

  @Override
  public void periodic() {
    // periodicReset();

    // FRONT_LEFT_ENC.setDouble(frontLeft.turningEncoder.getAbsolutePosition().refresh().getValue()
    // * 360);
    // FRONT_RIGHT_ENC.setDouble(frontRight.turningEncoder.getAbsolutePosition().refresh().getValue()
    // * 360);
    // BACK_LEFT_ENC.setDouble(backLeft.turningEncoder.getAbsolutePosition().refresh().getValue()
    // * 360);
    // BACK_RIGHT_ENC.setDouble(backRight.turningEncoder.getAbsolutePosition().refresh().getValue()
    // * 360);

    // FRONT_LEFT_ENC.setDouble(SwerveModule.normalizeAngle2(frontLeft.turningNeoEncoder.getPosition())
    // * (180 / Math.PI));
    // FRONT_RIGHT_ENC
    // .setDouble(SwerveModule.normalizeAngle2(frontRight.turningNeoEncoder.getPosition())
    // * (180 / Math.PI));
    // BACK_LEFT_ENC.setDouble(SwerveModule.normalizeAngle2(backLeft.turningNeoEncoder.getPosition())
    // * (180 / Math.PI));
    // BACK_RIGHT_ENC.setDouble(SwerveModule.normalizeAngle2(backRight.turningNeoEncoder.getPosition())
    // * (180 / Math.PI));

    frontLeftDriveEncoder.setDouble(frontLeft.getPosition().distanceMeters);
    backLeftDriveEncoder.setDouble(backLeft.getPosition().distanceMeters);
    frontRightDriveEncoder.setDouble(frontRight.getPosition().distanceMeters);
    backRightDriveEncoder.setDouble(backRight.getPosition().distanceMeters);

    frontLeftTurnOutput.setDouble(frontLeft.getMainTurnOutput());
    backLeftTurnOutput.setDouble(backLeft.getMainTurnOutput());
    frontRightTurnOutput.setDouble(frontRight.getMainTurnOutput());
    backRightTurnOutput.setDouble(backRight.getMainTurnOutput());

    globalAngle.setDouble(Robot.navX.getAngle() % 360);
    angleVelo.setDouble(Robot.navX.getRate());
    updateOdometry();
    Logger.recordOutput("velocity", Robot.navX.getVelocityY());

    Logger.recordOutput("Odometry", getPose());
    Logger.recordOutput("angular velocity", Robot.navX.getRate());
    Logger.recordOutput("Front Left Arya", frontLeft.getPosition().angle.getDegrees());
    Logger.recordOutput("Back Left Arav", backLeft.getPosition().angle.getDegrees());
    Logger.recordOutput("Front Right Alan", frontRight.getPosition().angle.getDegrees());
    Logger.recordOutput("Back Right Aritra", backRight.getPosition().angle.getDegrees());
    Logger.recordOutput("Alan is a persecuter", true);
    Logger.recordOutput("Real Swerve Module States", getModuleStates());
    if(frontLeftVisionPose!=null){
    Logger.recordOutput("frontLeftVisionPose",frontLeftVisionPose.estimatedPose);
    }
    if(frontRightVisionPose!=null){
    Logger.recordOutput("frontRightVisionPose",frontLeftVisionPose.estimatedPose);
    }
  }
}