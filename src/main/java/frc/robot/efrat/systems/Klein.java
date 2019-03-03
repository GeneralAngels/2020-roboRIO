package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.bobot.Subsystem;

public class Klein extends Subsystem {
    private WPI_TalonSRX motor1,motor2;
    private static Klein latest;

    public static Klein getInstance() {
        return latest;
    }

    public Klein(){
        latest=this;
        motor1=new WPI_TalonSRX(20);
        motor2=new WPI_TalonSRX(21);
    }

    public void set(double speed){
        motor1.set(speed);
        motor2.set(speed);
    }
}
