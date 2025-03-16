package frc.subsystems;

import static frc.robot.Robot.coralState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import frc.commands.endeffector.FixCoral;
import frc.commands.endeffector.IntakeCoral;
import frc.robot.Robot.CoralState;
import frc.robot.RobotMap.EndEffector;

public class EndEffectorSubsystem extends SubsystemBase {

  public final SparkMax leftMotor;
  public final SparkMax rightMotor;
  public final LaserCan firstLC;
  public final LaserCan secondLC;
  public boolean retracted;
  public boolean indexed;
  public int stateCheck;
  Timer timer;
  public static final ShuffleboardTab EE_TAB = Shuffleboard.getTab("End Effector");

  public static final GenericEntry firstLCValue = EE_TAB.add("firstLC Value", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

  public static final GenericEntry secondLCValue = EE_TAB.add("secondLC Value", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();

  public EndEffectorSubsystem() {
    retracted = false;
    indexed = false;
    timer = new Timer();
    leftMotor = new SparkMax(EndEffector.LEFT_END_EFFECTOR_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(EndEffector.RIGHT_END_EFFECTOR_MOTOR_ID, MotorType.kBrushless);

    firstLC = new LaserCan(EndEffector.FIRST_LC_ID);
    secondLC = new LaserCan(EndEffector.SECOND_LC_ID);

    try {
      secondLC.setRangingMode(LaserCan.RangingMode.SHORT);
      secondLC.setRegionOfInterest(new LaserCan.RegionOfInterest(12, 8, 4, 4));
      secondLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    try {
      firstLC.setRangingMode(LaserCan.RangingMode.SHORT);
      firstLC.setRegionOfInterest(new LaserCan.RegionOfInterest(2, 8, 4, 4));
      firstLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

  }

  public boolean firstSeeCoral() {
    return firstLC.getMeasurement().distance_mm < 250;
  }

  public boolean secondSeeCoral() {
    return secondLC.getMeasurement().distance_mm < 80;
  }

  @Override
  public void periodic() {
    firstLCValue.setDouble(firstLC.getMeasurement().distance_mm);
    secondLCValue.setDouble(secondLC.getMeasurement().distance_mm);
    if (!indexed && !retracted && coralState == CoralState.HOLDING && firstSeeCoral()) {
      leftMotor.set(0.15);
      rightMotor.set(-0.15);
      stateCheck=1;
    } else if (indexed && !retracted && coralState == CoralState.HOLDING) {
      if (timer.get() == 0.0) {
        timer.start();
      }
      leftMotor.set(-0.15);
      rightMotor.set(0.15);
      if (!timer.hasElapsed(0.4)) {
        stateCheck=3;
      } else {
        retracted = true;
        timer.reset();
        timer.stop();
        leftMotor.set(0);
        rightMotor.set(0);
        stateCheck=4;
      }

    } else if (coralState == CoralState.HOLDING) {
      if(!indexed) {
        leftMotor.set(0);
        rightMotor.set(0);
      }
      indexed = true;
      stateCheck=2;
    } else if (coralState == CoralState.NOT_HOLDING) {
      retracted = false;
      indexed = false;
    }
    Logger.recordOutput("stateCheck", stateCheck);
    Logger.recordOutput("coralState", coralState.toString());

    /*
     * operatorController.a().toggleOnTrue(new GoToPosition(4));
     * operatorController.b().toggleOnTrue(new GoToPosition(3));
     * operatorController.x().toggleOnTrue(new GoToPosition(2));
     * operatorController.y().toggleOnTrue(new GoToPosition(1));
     */
  }

}
