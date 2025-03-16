package frc.commands.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import static frc.robot.Robot.elevator;

public class GoHome extends Command {

    int newLevel;
    boolean state;
    
    public GoHome() {
        addRequirements(Robot.elevator);
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
        elevator.leftElevatorMotor.set(0);
        elevator.rightElevatorMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        if(state) {
        return  newLevel == 1; //|| (double)Robot.elevator.getRealPosition(Robot.elevator.rightElevatorMotor) - (double)elevator.position > 0.05;
        } else {
            return  newLevel == 1 || Math.abs((double)Robot.elevator.getRealPosition(Robot.elevator.rightElevatorMotor) - (double)elevator.position) < 0.05;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("end");
    }
}