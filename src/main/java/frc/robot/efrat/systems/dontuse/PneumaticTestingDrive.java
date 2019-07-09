package frc.robot.efrat.systems.dontuse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.drive.Gyroscope;
import org.json.JSONObject;

public class PneumaticTestingDrive extends DifferentialDrive<Victor> {

    private static final String LATESTX = "padx", LATESTY = "pady";
    protected Gyroscope gyro;
    private boolean pneumatics = false;
    private DoubleSolenoid gear1;
    private double latestX, latestY;

    public PneumaticTestingDrive() {
        if (pneumatics) {
            gear1 = new DoubleSolenoid(6, 7);
        }
        right.addMotor(new Victor(0));
        right.addMotor(new Victor(1));
        left.addMotor(new Victor(2));
        left.addMotor(new Victor(3));
        left.setEncoder(new Encoder(7, 6));
        right.setEncoder(new Encoder(4, 5));
//        right.setDirection(MotorGroup.BACKWARD);
//        left.getEncoder().reset();
//        right.getEncoder().reset();
        gyro = new Gyroscope();
        initGyro(gyro);

    }

    @Override
    public JSONObject toJSON() {
        JSONObject json = super.toJSON();
        json.put(LATESTX, latestX);
        json.put(LATESTY, latestY);
        return json;
    }

    public void setBench(double speed, double turn) {
        direct(speed, turn);
    }

    public void gearUp() {
        if (pneumatics) {
            gear1.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void gearDown() {
        if (pneumatics) {
            gear1.set(DoubleSolenoid.Value.kReverse);
        }
    }

    @Override
    public void set(double speed, double turn, boolean auto) {
        latestX = turn;
        latestY = speed;
        super.set(speed, turn, auto);
    }
}
