package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.base.control.PID;

public class KobiShooter extends frc.robot.base.Module {

    private WPI_TalonSRX motor;
    private PID motorControlVelocity;

    public KobiShooter() {
        super("shooter");
        motor = new WPI_TalonSRX(20); // TODO config in phonix
        motorControlVelocity = new PID("motorControlVelocity", 0 , 0, 0, 0);

        command("shoot", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return "NI";
            }
        });
        command("move", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return "NI";
            }
        });
    }

    public void applyPower(double power){
        motor.set(power);
    }

    public void setVelocity(double velocity){
//        motorControlVelocity.PIDVelocity();
    }

    public int getPosition(){
        return motor.getSelectedSensorPosition();
    }
}
