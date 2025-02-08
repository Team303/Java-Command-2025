package frc.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.EndEffector;

public class EndEffectorSubsystem extends SubsystemBase {
    
    public final SparkMax leftMotor;
    public final SparkMax rightMotor;
    public final LaserCan firstLC;
    public final LaserCan secondLC;

    public EndEffectorSubsystem() {
      leftMotor = new SparkMax(EndEffector.LEFT_END_EFFECTOR_MOTOR_ID, MotorType.kBrushless);
      rightMotor = new SparkMax(EndEffector.RIGHT_END_EFFECTOR_MOTOR_ID, MotorType.kBrushless);

      firstLC = new LaserCan(EndEffector.FIRST_LC_ID);
      secondLC = new LaserCan(EndEffector.SECOND_LC_ID);

      try {
        secondLC.setRangingMode(LaserCan.RangingMode.SHORT);
        secondLC.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        secondLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      } catch (ConfigurationFailedException e) {
       System.out.println("Configuration failed! " + e);
      }


    try {
      firstLC.setRangingMode(LaserCan.RangingMode.SHORT);
      firstLC.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      firstLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
      }


    }

    public boolean firstSeeCoral()
    {
      return firstLC.getMeasurement().distance_mm < 20;
    }

    public boolean secondSeeCoral() {
      return secondLC.getMeasurement().distance_mm < 20;
    }
    
  

}
