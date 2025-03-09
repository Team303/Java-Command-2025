package frc.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.fasterxml.jackson.databind.deser.std.StackTraceElementDeserializer.Adapter;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.RobotMap;
import frc.robot.RobotMap.Algae;
import static frc.robot.Robot.algae;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase{
//     public final SparkMax intakeMotor;
//     public final SparkMax pivotMotor;
//     public final AbsoluteEncoder enc;
//    public final PIDController pidController;
//     public final ArmFeedforward pivotFeedforward;

//     public AlgaeSubsystem() {
//         intakeMotor = new SparkMax(Algae.INTAKE_MOTOR_ID, MotorType.kBrushless);
//         pivotMotor = new SparkMax(Algae.PIVOT_MOTOR_ID, MotorType.kBrushless);
//         enc = intakeMotor.getAbsoluteEncoder();
//         pivotFeedforward = new ArmFeedforward(RobotMap.Algae.PIVOT_FEED_FORWARD_KS,
// 				RobotMap.Algae.PIVOT_FEED_FORWARD_KG,
// 				RobotMap.Algae.PIVOT_FEED_FORWARD_KV,
// 				RobotMap.Algae.PIVOT_FEED_FORWARD_KA);
        

//         pidController = new PIDController(5, 0, 0);



//         SparkMaxConfig x = new SparkMaxConfig();
//         x.idleMode(SparkBaseConfig.IdleMode.kBrake);
//         pivotMotor.configure(x, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

//         // double pivotOutput = pidController.calculate()

//     }

//     public double normalizeAngle(double AngleRad)
//     {
//         AngleRad %= Math.PI*2;

//         if(AngleRad < 0)
//         {
//             AngleRad += Math.PI*2;
//         }

//         return AngleRad;
//     }

//     // public double getAbsoluteAngle()
//     // {
//     //     // return normalizeAngle(.getAbsolutePosition()*2*Math.PI - Math.toRadians(47+308));
//     // }
    

}
