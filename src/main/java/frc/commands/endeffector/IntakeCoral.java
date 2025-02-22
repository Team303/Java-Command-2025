package frc.commands.endeffector;

import static frc.robot.Robot.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;


public class IntakeCoral extends Command {
    
    public IntakeCoral() {
        addRequirements(Robot.endEffector);
    }

    public void execute() {
        endEffector.leftMotor.set(0.5);
        endEffector.rightMotor.set(-0.5);
    }

   public boolean isFinished() {
       return endEffector.secondSeeCoral();
   }
    
    public void end(boolean interrupted) {
        endEffector.leftMotor.set(0);
        endEffector.rightMotor.set(0);
    }
}
