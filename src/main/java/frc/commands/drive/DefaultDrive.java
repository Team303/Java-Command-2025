package frc.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.subsystems.DriveSubsystem;

public class DefaultDrive extends Command {
    boolean fieldOriented;
    Translation2d translation;
    double percentPower;

    boolean isAlliance = true;

    public DefaultDrive(boolean fieldOriented) {
        addRequirements(Robot.swerve);
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void execute() {
        percentPower = (1 - Robot.driverController.getLeftTriggerAxis() * 0.3);
       // System.out.println("Percentpower: " + percentPower);
        Translation2d translation = new Translation2d(
                MathUtil.applyDeadband(-Robot.driverController.getLeftY(), 0.25) * DriveSubsystem.kMaxSpeed
                        * percentPower * 0.3,
                MathUtil.applyDeadband(-Robot.driverController.getLeftX(), 0.25) * DriveSubsystem.kMaxSpeed
                        * percentPower * 0.3);

        double rotation = -MathUtil.applyDeadband(Robot.driverController.getRightX() * percentPower, 0.2)
                * DriveSubsystem.kMaxAngularSpeed * percentPower;

      //  System.out.println("rotation: " + rotation);

        Robot.swerve.drive(translation, rotation, fieldOriented);
    }
}