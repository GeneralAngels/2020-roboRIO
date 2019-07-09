package frc.robot.efrat.systems.dontuse;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.drive.Drivebox;
import frc.robot.base.drive.Gyroscope;
import org.json.JSONObject;

public class PneumaticDrive extends DifferentialDrive<WPI_TalonSRX> {
    private static final String LATESTX = "padx", LATESTY = "pady";
    private static PneumaticDrive latest;
    protected Gyroscope gyro;
    private boolean pneumatics = false;
    private double latestX, latestY;
    private DoubleSolenoid gear;

    public PneumaticDrive() {
        latest = this;
        if (pneumatics) {
            gear = new DoubleSolenoid(6, 7);
        }
        right.add(new WPI_TalonSRX(3));
        right.add(new WPI_TalonSRX(4));
        left.add(new WPI_TalonSRX(1));
        left.add(new WPI_TalonSRX(2));
        left.setEncoder(new Encoder(4,5));
        right.setEncoder(new Encoder(2,3));
        left.setDirection(Drivebox.DIRECTION_BACKWARD);
//        right.setDirection(Drivebox.DIRECTION_BACKWARD);
        //left.getEncoder().reset();
        //right.getEncoder().reset();
//        gyro = new Gyroscope();
//        initGyro(gyro);

    }

    public static PneumaticDrive getInstance() {
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
    public void set(double speed, double turn, boolean auto) {
        latestX = turn;
        latestY = speed;
        super.set(speed, turn, auto);
    }
}
