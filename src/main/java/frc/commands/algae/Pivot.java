package frc.commands.algae;
import static frc.robot.Robot.algae;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class Pivot extends Command{

double theta0;
double diff_theta;
    
public Pivot() {
    // addRequirements(algae);
}

public void initialize() {
    // theta0 = algae.pivotMotor.getAbsoluteEncoder().getPosition();
}

public void execute(double theta_Final) {
    // diff_theta = theta_Final-(algae.pivotMotor.getAbsoluteEncoder().getPosition());
    // algae.pivotMotor.set(RobotMap.Algae.PIVOT_MOTOR_Kp*diff_theta);

}

// public boolean isFinished() {
//     // return Math.abs(diff_theta) < 1;
// }

}
