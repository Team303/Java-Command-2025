package frc.commands.endeffector;

import static frc.robot.Robot.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.CoralState;
import static frc.robot.Robot.coralState;
import org.littletonrobotics.junction.Logger;


public class IntakeCoral extends Command {
    // boolean canSee=false;
    
    public IntakeCoral() {
        addRequirements(Robot.endEffector);
    }

    public void execute() {
        endEffector.leftMotor.set(0.45);
        endEffector.rightMotor.set(-0.45);
        // canSee=endEffector.firstSeeCoral();
        // System.out.println("Yoo");
        
    }

   public boolean isFinished() {
       return endEffector.secondSeeCoral();
   }
    
    public void end(boolean interrupted) {
        coralState = CoralState.HOLDING;
        endEffector.leftMotor.set(0);
        endEffector.rightMotor.set(0);
    }
}
