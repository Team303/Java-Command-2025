package frc.commands.algae;

import static frc.robot.Robot.algae;
import static frc.robot.Robot.operatorController;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    
    public Intake() {
        addRequirements(algae);
    }

    public void initialize() {

    }

    public void execute() {
        // algae.intakeMotor.set(1);
    }

    // public boolean isFinished() {
    //     return !operatorController.a().getAsBoolean();
    // }
}
