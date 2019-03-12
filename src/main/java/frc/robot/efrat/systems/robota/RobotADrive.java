package frc.robot.efrat.systems.robota;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.bobot.drive.DifferentialDrive;
import frc.robot.bobot.drive.Drivebox;
import frc.robot.bobot.drive.Gyroscope;
import org.json.JSONObject;

public class RobotADrive extends DifferentialDrive<WPI_TalonSRX> {
    private static RobotADrive latest;
    protected AHRS gyro;
    private boolean pneumatics = true;
    private double latestX, latestY;
    private DoubleSolenoid gear;

    public RobotADrive() {
        latest = this;
        if (pneumatics) {
            gear = new DoubleSolenoid(6, 7);
        }
        // ROBOT B
        // TODO change on robotA
        right.add(new WPI_TalonSRX(10));
        left.add(new WPI_TalonSRX(11));
        left.add(new WPI_TalonSRX(12));
        right.add(new WPI_TalonSRX(13));
        // ROBOT A
//        right.add(new WPI_TalonSRX(10));
//        right.add(new WPI_TalonSRX(11));
//        left.add(new WPI_TalonSRX(12));
//        left.add(new WPI_TalonSRX(13));
        left.setEncoder(new Encoder(6, 7));
        right.setEncoder(new Encoder(9, 8));
//        left.setDirection(Drivebox.DIRECTION_BACKWARD);
        right.setDirection(Drivebox.DIRECTION_BACKWARD);
        left.getEncoder().reset();
        right.getEncoder().reset();
        gyro = new Gyroscope();
        initGyro(gyro);
    }

    public static RobotADrive getInstance() {
        return latest;
    }

    public void setBench(double speed, double turn) {
        direct(speed, turn);
    }

    public void gearUp() {
        if (pneumatics) {
            gear.set(DoubleSolenoid.Value.kForward);
            gearRatio = 14.0 / 60.0;
//            log("Gear: UP");
        }
    }

    public void gearDown() {
        if (pneumatics) {
            gear.set(DoubleSolenoid.Value.kReverse);
            gearRatio = 20.0 / 44.0;
//            log("Gear: DOWN");
        }
    }

    @Override
    public JSONObject toJSON() {
        JSONObject o = super.toJSON();
        if (gear != null) o.put("gear", gear.get() == DoubleSolenoid.Value.kForward);
        return o;
    }

    @Override
    public void set(double speed, double turn) {
        latestX = turn;
        latestY = speed;
        super.set(speed, turn);
//        log("EL: "+left.getEncoder().getRaw()+" ER: "+right.getEncoder().getRaw());
    }
}
