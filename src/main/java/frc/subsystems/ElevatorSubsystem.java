package frc.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.Elevator;

public class ElevatorSubsystem extends SubsystemBase {
    public final TalonFX leftElevatorMotor;
    public final TalonFX rightElevatorMotor;

      public static final ShuffleboardTab ELEVATOR_TAB = Shuffleboard.getTab("Elevator");

      public static final GenericEntry leftMotorRotations = ELEVATOR_TAB.add("left motor rotations", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

      public static final GenericEntry rightMotorRotations = ELEVATOR_TAB.add("right motor rotations", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();



    public ElevatorSubsystem() {
        leftElevatorMotor = new TalonFX(Elevator.LEFT_ELEVATOR_MOTOR_ID);
        rightElevatorMotor = new TalonFX(Elevator.RIGHT_ELEVATOR_MOTOR_ID);

        leftElevatorMotor.setInverted(false);
        rightElevatorMotor.setInverted(true);

        var LeftElevatorMotorTalonFXConfigurator = leftElevatorMotor.getConfigurator();
        var RightElevatorMotorTalonFXConfigurator = rightElevatorMotor.getConfigurator();
 
        var limitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 60;
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = 50;
        limitConfigs.SupplyCurrentLimitEnable = true;

        LeftElevatorMotorTalonFXConfigurator.apply(limitConfigs);
        RightElevatorMotorTalonFXConfigurator.apply(limitConfigs);


        var TalonFXConfiguration = new TalonFXConfiguration();

        var Slot0Configs = TalonFXConfiguration.Slot0;

        Slot0Configs.kG = 0.09;
        Slot0Configs.kV = 11.2;
        Slot0Configs.kA = 0.01;
        Slot0Configs.kP = 1;
        Slot0Configs.kI = 0;
        Slot0Configs.kD = 0;

        var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 2000;
        motionMagicConfigs.MotionMagicJerk = 0;
        motionMagicConfigs.MotionMagicCruiseVelocity = 200;

        

        leftElevatorMotor.getConfigurator().apply(TalonFXConfiguration);
        rightElevatorMotor.getConfigurator().apply(TalonFXConfiguration);


    }

    public void moveToSetpoint(double targetPos) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        leftElevatorMotor.setControl(m_request.withPosition(targetPos));
        rightElevatorMotor.setControl(m_request.withPosition(-targetPos));
    }

    @Override
    public void periodic() { 
    leftMotorRotations.setDouble(leftElevatorMotor.getPosition().getValueAsDouble());
    rightMotorRotations.setDouble(leftElevatorMotor.getPosition().getValueAsDouble());

  }
}
