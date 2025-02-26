package frc.commands.endeffector;

import static frc.robot.Robot.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.CoralState;
import static frc.robot.Robot.coralState;


public class FixCoral extends Command {
    
    public FixCoral() {
        addRequirements(Robot.endEffector);
    }

    public void execute() {
        endEffector.leftMotor.set(0.45);
        endEffector.rightMotor.set(-0.45);
        System.out.println("Yoo2");

        
    }

//    public boolean isFinished() {
//        return (!endEffector.firstSeeCoral());
//    }
    
//     public void end(boolean interrupted) {
//         endEffector.leftMotor.set(0);
//         endEffector.rightMotor.set(0);
//     }
}
