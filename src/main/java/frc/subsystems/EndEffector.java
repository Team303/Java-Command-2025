package frc.subsystems;

import au.grapplerobotics.*;;


public class EndEffector {

    private LaserCan motor1;
    private LaserCan motor2;

    public void CoralOutTake( int idCan1, int idCan2){
        motor1 = new LaserCan(idCan1);
        motor2 = new LaserCan(idCan2);
    }

    public void setSpeed(double speed){
        motor1.set(speed);

    }





    

    


}
