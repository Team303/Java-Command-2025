package frc.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.EndEffector;

public class EndEffectorSubsystem extends SubsystemBase {
    
    public final SparkMax leftMotor;
    public final SparkMax rightMotor;
    public final LaserCan firstLC;
    public final LaserCan secondLC;

     public static final ShuffleboardTab EE_TAB = Shuffleboard.getTab("End Effector");

      public static final GenericEntry firstLCValue = EE_TAB.add("firstLC Value", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

      public static final GenericEntry secondLCValue = EE_TAB.add("secondLC Value", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

    public EndEffectorSubsystem() {
      leftMotor = new SparkMax(EndEffector.LEFT_END_EFFECTOR_MOTOR_ID, MotorType.kBrushless);
      rightMotor = new SparkMax(EndEffector.RIGHT_END_EFFECTOR_MOTOR_ID, MotorType.kBrushless);

      firstLC = new LaserCan(EndEffector.FIRST_LC_ID);
      secondLC = new LaserCan(EndEffector.SECOND_LC_ID);

      try {
        secondLC.setRangingMode(LaserCan.RangingMode.SHORT);
        secondLC.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
        secondLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      } catch (ConfigurationFailedException e) {
       System.out.println("Configuration failed! " + e);
      }


    try {
      firstLC.setRangingMode(LaserCan.RangingMode.LONG);
      firstLC.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
      firstLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
      }


    }

    public boolean firstSeeCoral()
    {
      return firstLC.getMeasurement().distance_mm < 120;
    }

    public boolean secondSeeCoral() {
      return secondLC.getMeasurement().distance_mm < 80;
    }

    @Override
    public void periodic() { 
    firstLCValue.setDouble(firstLC.getMeasurement().distance_mm);
    secondLCValue.setDouble(secondLC.getMeasurement().distance_mm);
/*
 * 	operatorController.a().toggleOnTrue(new GoToPosition(4));
		operatorController.b().toggleOnTrue(new GoToPosition(3));
		operatorController.x().toggleOnTrue(new GoToPosition(2));
		operatorController.y().toggleOnTrue(new GoToPosition(1));
 */
  }
    
  

}
