package frc.commands.endeffector;
import static frc.robot.Robot.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.CoralState;
import static frc.robot.Robot.coralState;

public class ShootCoral extends Command {
    
    int level;
    public ShootCoral(int level) {
        addRequirements(Robot.endEffector);
        this.level = level;
        coralState = CoralState.NOT_HOLDING;
    }

    public void execute() {
        if (this.level == 1 ) {
            Robot.endEffector.rightMotor.set(-0.07);
            Robot.endEffector.leftMotor.set(0.2);
        }
        else {
            Robot.endEffector.rightMotor.set(-0.4);
            Robot.endEffector.leftMotor.set(0.4);
        }
    }

    public boolean isFinished() {
        return !Robot.endEffector.secondSeeCoral();
    }
 public void end(boolean interrupted) {
        coralState = CoralState.NOT_HOLDING;
        endEffector.leftMotor.set(0);
        endEffector.rightMotor.set(0);
    }

} 
