package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.bobot.drive.DifferentialDrive;
import frc.robot.bobot.drive.Drivebox;
import frc.robot.bobot.drive.Gyroscope;
import org.json.JSONObject;

public class PneumaticDriveSpark extends DifferentialDrive<Spark> {
    private static final String LATESTX = "padx", LATESTY = "pady";
    private static PneumaticDriveSpark latest;
    protected Gyroscope gyro;
    private boolean pneumatics = true;
    private double latestX, latestY;
    private DoubleSolenoid gear;

    public PneumaticDriveSpark() {
        latest = this;
        if (pneumatics) {
            gear = new DoubleSolenoid(6, 7);
        }
        right.add(new Spark(10));
        right.add(new Spark(11));
        left.add(new Spark(12));
        left.add(new Spark(13));
//        left.setEncoder(new Encoder(7, 6));
//        right.setEncoder(new Encoder(4, 5));
        left.setDirection(Drivebox.DIRECTION_BACKWARD);
        right.setDirection(Drivebox.DIRECTION_BACKWARD);
//        left.getEncoder().reset();
//        right.getEncoder().reset();
//        gyro = new Gyroscope();
//        initGyro(gyro);

    }

    public static PneumaticDriveSpark getInstance() {
        return latest;
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
            gear.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void gearDown() {
        if (pneumatics) {
            gear.set(DoubleSolenoid.Value.kReverse);
        }
    }

    @Override
    public void set(double speed, double turn) {
        latestX = turn;
        latestY = speed;
        super.set(speed, turn);
    }
}
