package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.base.control.PID;

public class KobiShooter extends frc.robot.base.Module {

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private WPI_TalonSRX motor3;
    private PID motorsControlVelocity;

    public KobiShooter() {
        super("shooter");

        motor1 = new WPI_TalonSRX(20); // TODO config in phonix
        motor2 = new WPI_TalonSRX(21); // TODO config in phonix
        motor3 = new WPI_TalonSRX(22); // TODO config in phonix

        motorsControlVelocity = new PID("motorControlVelocity", 0 , 0, 0, 0);

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
        motor1.set(power);
        motor2.set(power);
        motor3.set(-power);
    }

    public void setVelocity(double velocity){
//        motorsControlVelocity.
    }

    public int getPosition(){
        return motor1.getSelectedSensorPosition();
    }
}
