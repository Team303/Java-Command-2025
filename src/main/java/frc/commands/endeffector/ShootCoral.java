package frc.commands.endeffector;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShootCoral extends Command {
    
    int level;
    public ShootCoral(int level) {
        addRequirements(Robot.endEffector);
        this.level = level;
    }

    public void execute() {
        if (this.level == 1 ) {
            Robot.endEffector.rightMotor.set(0.3);
            Robot.endEffector.leftMotor.set(-0.6);
        }
        else {
            Robot.endEffector.rightMotor.set(0.6);
            Robot.endEffector.leftMotor.set(-0.6);
        }
    }

    public boolean isFinished() {
        return !Robot.endEffector.secondSeeCoral();
    }
} 
