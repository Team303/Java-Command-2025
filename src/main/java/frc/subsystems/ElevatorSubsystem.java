package frc.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Elevator;

public class ElevatorSubsystem extends SubsystemBase {
    public final TalonFX leftElevatorMotor;
    public final TalonFX rightElevatorMotor;

      public static final ShuffleboardTab ELEVATOR_TAB = Shuffleboard.getTab("Elevator");

      public static final GenericEntry leftMotorRotations = ELEVATOR_TAB.add("left motor rotations", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

      public static final GenericEntry rightMotorRotations = ELEVATOR_TAB.add("right motor rotations", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

      public static final GenericEntry robotLevel = ELEVATOR_TAB.add("level", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

      public static final GenericEntry controls = ELEVATOR_TAB.add("controls", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

      public static final GenericEntry goalPosition = ELEVATOR_TAB.add("goal", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();
      public int level;
      public double position;
  



    public ElevatorSubsystem() {
        leftElevatorMotor = new TalonFX(Elevator.LEFT_ELEVATOR_MOTOR_ID);
        rightElevatorMotor = new TalonFX(Elevator.RIGHT_ELEVATOR_MOTOR_ID);

        // leftElevatorMotor.setInverted(true);
        // rightElevatorMotor.setInverted(false);
        var TalonFXConfiguration = new TalonFXConfiguration();

        var Slot0Configs = TalonFXConfiguration.Slot0;
        var Slot1Configs = TalonFXConfiguration.Slot1;
        Slot0Configs.kS= 0;
        Slot1Configs.kS = Slot0Configs.kS;
        Slot0Configs.kG = 0.45;
        Slot1Configs.kG = Slot0Configs.kG;
        Slot0Configs.kV = -0.3;
        Slot1Configs.kV = -Slot0Configs.kV;
        Slot0Configs.kA = -0.1;
        Slot1Configs.kA = Slot0Configs.kA;
        Slot0Configs.kP = 14*0.8;
        Slot1Configs.kP = Slot0Configs.kP;
        Slot0Configs.kI = 0;
        Slot1Configs.kI = Slot0Configs.kI;
        Slot0Configs.kD = 0;
        Slot1Configs.kD = Slot0Configs.kD;


        var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 35;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60;
        leftElevatorMotor.getConfigurator().apply(TalonFXConfiguration);
        rightElevatorMotor.getConfigurator().apply(TalonFXConfiguration);


        
        leftElevatorMotor.setPosition(0);
        rightElevatorMotor.setPosition(0); 
        leftElevatorMotor.getConfigurator().apply(TalonFXConfiguration);
        rightElevatorMotor.getConfigurator().apply(TalonFXConfiguration);

        var LeftElevatorMotorTalonFXConfigurator = leftElevatorMotor.getConfigurator();
        var RightElevatorMotorTalonFXConfigurator = rightElevatorMotor.getConfigurator();
 
        var limitConfigs = new CurrentLimitsConfigs();
        var leftMotorConfigs = new MotorOutputConfigs();
        var rightMotorConfigs = new MotorOutputConfigs();
        leftMotorConfigs.Inverted= InvertedValue.Clockwise_Positive;
        rightMotorConfigs.Inverted= InvertedValue.CounterClockwise_Positive;
        leftMotorConfigs.NeutralMode= NeutralModeValue.Brake;
        rightMotorConfigs.NeutralMode= NeutralModeValue.Brake;


        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 120 ;
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = 120;
        limitConfigs.SupplyCurrentLimitEnable = true;

        LeftElevatorMotorTalonFXConfigurator.apply(leftMotorConfigs);
        RightElevatorMotorTalonFXConfigurator.apply(rightMotorConfigs);
        LeftElevatorMotorTalonFXConfigurator.apply(limitConfigs);
        RightElevatorMotorTalonFXConfigurator.apply(limitConfigs);


        // Slot0Configs.kG = 0.09;
        // Slot0Configs.kV = 11.2;
        // Slot0Configs.kA = 0.01;
        // Slot0Configs.kP = 1;
        // Slot0Configs.kI = 0;
        // Slot0Configs.kD = 0;

        // var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
        // motionMagicConfigs.MotionMagicAcceleration = 2000;
        // motionMagicConfigs.MotionMagicJerk = 0;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 200;
      

        // leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        // rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        level = 1;
        position = 0;

    }

    public void moveToSetpoint(double targetPos) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(targetPos);
        System.out.println(targetPos);
        leftElevatorMotor.setControl(m_request.withPosition((targetPos)).withSlot(1));
        rightElevatorMotor.setControl(m_request.withPosition((targetPos)).withSlot(1));
    }

    public void update() {
      // position = leftElevatorMotor.getClosedLoopReference().getValueAsDouble();
    }

    public void configuration(TalonFXConfiguration newConfig) {
      leftElevatorMotor.getConfigurator().apply(newConfig);
      rightElevatorMotor.getConfigurator().apply(newConfig);
    }

    public double getRealPosition(TalonFX motor) {
      return motor.getPosition().getValueAsDouble(); // - RobotMap.Elevator.ELEVATOR_ENCODER_OFFSET;
    }



    @Override
    public void periodic() { 
    rightMotorRotations.setDouble(getRealPosition(rightElevatorMotor));
    robotLevel.setInteger(level);
    goalPosition.setDouble(position);
    controls.setString("(4A)(3B)(2X)(1Y)");
/*
 * 	operatorController.a().toggleOnTrue(new GoToPosition(4));
		operatorController.b().toggleOnTrue(new GoToPosition(3));
		operatorController.x().toggleOnTrue(new GoToPosition(2));
		operatorController.y().toggleOnTrue(new GoToPosition(1));
 */
  }
}