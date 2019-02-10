package frc.robot.efrat.systems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import frc.robot.bobot.drive.DifferentialDrive;
import frc.robot.bobot.drive.Drivebox;
import frc.robot.bobot.drive.Gyroscope;
import org.json.JSONObject;

public class PneumaticTestingDrive extends DifferentialDrive<Victor> {

    private static final String LATESTX = "padx", LATESTY = "pady";
    protected Gyroscope gyro;
    private boolean pneumatics = true;
    private DoubleSolenoid gear1, gear2;
    private double latestX, latestY;

    public PneumaticTestingDrive() {
//        ENCODER_COUNT_PER_REVOLUTION = 8192;
//        MAX_V = 0.5;
//        MAX_OMEGA = 0;
        if (pneumatics) {
            gear1 = new DoubleSolenoid(4, 5);
//            gear2 = new DoubleSolenoid(6, 7);
        }
        right.add(new Victor(0), new Victor(1));
        left.add(new Victor(2), new Victor(3));
        left.setEncoder(new Encoder(7, 6));
        right.setEncoder(new Encoder(4, 5));
        left.setDirection(Drivebox.DIRECTION_BACKWARD);
//        left.getEncoder().reset();
//        right.getEncoder().reset();
        gyro = new Gyroscope();
        initGyro(gyro);

    }

    @Override
    public JSONObject toJSON() {
//        log("ODOMETRY UPDATE");
//        odometry.calculate((right.getEncoder().get()/ENCODER_COUNT_PER_REVOLUTION)*(2*WHEEL_RADIUS*Math.PI),(left.getEncoder().get()/ENCODER_COUNT_PER_REVOLUTION)*(2*WHEEL_RADIUS*Math.PI),gyro.getAngle());
//        log(odometry.toJSON().toString());
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
            gear2.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void gearDown() {
        if (pneumatics) {
            gear1.set(DoubleSolenoid.Value.kReverse);
            gear2.set(DoubleSolenoid.Value.kReverse);
        }
    }

    @Override
    public void set(double speed, double turn) {
        latestX = turn;
        latestY = speed;
        super.set(speed, turn);
    }
}
