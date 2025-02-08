package frc.commands.drive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.FieldPosition;
import frc.robot.RobotMap.FieldConstants;

import static frc.robot.Robot.operatorControl;
import static frc.robot.Robot.swerve;

public class GoToReefPosition extends Command {

    FieldPosition position = FieldPosition.CURRENT_POSE;

    public GoToReefPosition() {
        addRequirements(swerve, operatorControl);
    }

    @Override
    public void initialize() {
        position = operatorControl.getQueuedPosition();

        for (int i = 0; i < 10; i++) {
            System.out.println(position);
        }
        operatorControl.lockIn();
    }

    @Override
    public void execute() {
        swerve.pathfindthenFollowPath(position);
        System.out.println("Going to " + position + "!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        operatorControl.lockOut();
    }
}
