package frc.commands.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import static frc.robot.Robot.elevator;

public class GoToPosition extends Command {

    int newLevel;
    
    public GoToPosition(int levelNumber) {
        addRequirements(Robot.elevator);
        newLevel = levelNumber;
    }

    @Override
    public void initialize() {

        // var TalonFXConfiguration = new TalonFXConfiguration();

        // var Slot0Configs = TalonFXConfiguration.Slot0;

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
        // elevator.configuration(TalonFXConfiguration);
      
    }

    
    @Override
    public void execute() {
        if(newLevel == 1) {
            Robot.elevator.moveToSetpoint(0);
           elevator.position = 0;
            elevator.level = 1;
        } else if (newLevel == 2) {
            Robot.elevator.moveToSetpoint(3.8);
           elevator.position = 3.8;
            elevator.level = 2;
        }
        else if (newLevel == 3) {
            Robot.elevator.moveToSetpoint(10.5);
           elevator.position = 10.5;
            elevator.level = 3;
        }
        else if (newLevel == 4) {
            Robot.elevator.moveToSetpoint(23);
           elevator.position = 23;
            elevator.level = 4;
        }
       Robot.elevator.update();
    }

    @Override
    public boolean isFinished() {
        return  newLevel == 1;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("end");
        elevator.leftElevatorMotor.set(0);
        elevator.rightElevatorMotor.set(0);
    }
}
