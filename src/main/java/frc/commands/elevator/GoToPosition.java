package frc.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class GoToPosition extends Command {
    
    public int level;
    public int position;

    public GoToPosition(int levelNumber) {
        addRequirements(Robot.elevator);
        level = levelNumber;
    }

    public void execute() {
        if(level == 1) {
            Robot.elevator.moveToSetpoint(0);
            position = 0;
        } else if (level == 2) {
            Robot.elevator.moveToSetpoint(10);
            position = 10;
        }
        else if (level == 3) {
            Robot.elevator.moveToSetpoint(20);
            position = 20;
        }
        else if (level == 4) {
            Robot.elevator.moveToSetpoint(30);
            position = 30;
        }
    }

    public boolean isFinished() {
        return Math.abs(Robot.elevator.leftElevatorMotor.getPosition().getValueAsDouble() - position) > 1.0;
    }
}
