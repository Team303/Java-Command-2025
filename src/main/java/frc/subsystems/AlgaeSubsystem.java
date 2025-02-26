package frc.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.fasterxml.jackson.databind.deser.std.StackTraceElementDeserializer.Adapter;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.RobotMap.Algae;
import static frc.robot.Robot.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase{
    public final SparkMax intakeMotor;
    public final SparkMax pivotMotor;
    public final SparkMaxAlternateEncoder adapter;


    public AlgaeSubsystem() {
        intakeMotor = new SparkMax(Algae.INTAKE_MOTOR_ID, MotorType.kBrushless);
        pivotMotor = new SparkMax(Algae.PIVOT_MOTOR_ID, MotorType.kBrushless);
        adapter = intakeMotor.SparkMaxAlternateEncoder(); 

        SparkMaxConfig x = new SparkMaxConfig();
        x.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotMotor.configure(x, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }
    

}
