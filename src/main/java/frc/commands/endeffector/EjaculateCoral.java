package frc.commands.endeffector;

import static frc.robot.Robot.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;


public class EjaculateCoral extends Command {
    
    public EjaculateCoral() {
        addRequirements(Robot.endEffector);
    }

    public void execute() {
        endEffector.leftMotor.set(-0.6);
        endEffector.rightMotor.set(0.6);
    }

    
    public void end(boolean interrupted) {
        endEffector.leftMotor.set(0);
        endEffector.rightMotor.set(0);
    }
}
