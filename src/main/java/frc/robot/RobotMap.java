package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class RobotMap {
	public static final class Swerve {


		//This should stay pretty similar but may change slightly
		/* CAN IDs of Drive Motors */
		public static final int LEFT_FRONT_DRIVE_ID = 2;
		public static final int RIGHT_FRONT_DRIVE_ID = 11;
		public static final int LEFT_BACK_DRIVE_ID = 5;
		public static final int RIGHT_BACK_DRIVE_ID = 8;

		/* Steer Encoder CAN IDs */
		public static final int LEFT_FRONT_STEER_CANCODER_ID = 3;
		public static final int RIGHT_FRONT_STEER_CANCODER_ID = 12;
		public static final int LEFT_BACK_STEER_CANCODER_ID = 6;
		public static final int RIGHT_BACK_STEER_CANCODER_ID = 9;

		/* CAN IDs of steer Motors turning */
		public static final int LEFT_FRONT_STEER_ID = 4;
		public static final int RIGHT_FRONT_STEER_ID = 13;
		public static final int LEFT_BACK_STEER_ID = 7;
		public static final int RIGHT_BACK_STEER_ID = 10;

		/* Steer Motor Offset */
		public static final double LEFT_BACK_STEER_OFFSET = (167.7-173.8+19.336+82+79.4-71.8-158.225+6.064-153.633+134.912) / 360;
		public static final double RIGHT_BACK_STEER_OFFSET = (35.4-9.5+52.9-8.5-153.105+11.9+6.592-145.898+55.723+76.73) / 360;
		public static final double LEFT_FRONT_STEER_OFFSET = (174.99-178.8-10.9+161.2-18.01+130.869-28.828+123.047-175.43) / 360;
		public static final double RIGHT_FRONT_STEER_OFFSET = (148.096-145.5+17.5-148.359+79.8-104.3-74.8+19.16+58.711+164.4) / 360;

		public static final double SWERVE_CONVERSION_FACTOR = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

		public static final double PHOTON_STDDEV_SCALING_FACTOR = 1.0;

		public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
		public static final double ROBOT_MOI = 3.0246; //kg m^2
		public static final double ROBOT_MASS = 52.965; //pounds

	}

	//TODO: Figure this out based off of the final design
	public static final class PhotonvisionConstants {
		public static final double FRONT_LEFT_CAMERA_HEIGHT_METERS = 0.2413;
		// public static final double BACK_RIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(8);
		// public static final double BACK_LEFT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(9);
		public static final double FRONT_RIGHT_CAMERA_HEIGHT_METERS = 0.2413; // NOT IFNAL
		public static final double GRID_TARGET_HEIGHT_METERS = 0.36;
		public static final double DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS = 0.59;
		public static final double BACK_CAMERA_PITCH_RADIANS = Math.toRadians(30);

		// public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA= new
		// Transform3d(new Translation3d(0.381, 0, FRONT_LEFT_CAMERA_HEIGHT_METERS),new
		// Rotation3d(0.0,0.0,Math.toRadians(45.0)));
		public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
				new Translation3d( Units.inchesToMeters(20),Units.inchesToMeters(-45),
						FRONT_LEFT_CAMERA_HEIGHT_METERS),
				new Rotation3d(180, Math.toRadians(18.6), Math.toRadians(21)));
		public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
				new Translation3d(Units.inchesToMeters(15),-Units.inchesToMeters(-10), FRONT_RIGHT_CAMERA_HEIGHT_METERS),
				new Rotation3d(Math.toRadians(180), Math.toRadians(18.6), Math.toRadians(-21)));
		// public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA= new
		// Transform3d(new Translation3d(0,-0.381,FRONT_RIGHT_CAMERA_HEIGHT_METERS),new
		// Rotation3d(0,0.0,Math.toRadians(315.0)));


	}
	//Subteam-specific constants go here
 
	

	//Field-specific constants go here
	//TODO: Change field-specific constants
	public static final class FieldConstants {
		public static double fieldLength = Units.inchesToMeters(684.223);
		public static double fieldWidth = Units.inchesToMeters(317);
		public static Pose2d centerSpeakOpenInBlue = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17),
				new Rotation2d(Units.radiansToDegrees(60)));

		public static Pose2d centerSpeakOpenInRed = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17),
		new Rotation2d(Units.radiansToDegrees(60)));
	}
}