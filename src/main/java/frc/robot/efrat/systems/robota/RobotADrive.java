package frc.robot.efrat.systems.robota;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.bobot.drive.DifferentialDrive;
import frc.robot.bobot.drive.Drivebox;
import frc.robot.bobot.drive.Gyroscope;

public class RobotADrive extends DifferentialDrive<WPI_TalonSRX> {
    private static RobotADrive latest;
    protected Gyroscope gyro;
    private boolean pneumatics = true;
    private double latestX, latestY;
    private DoubleSolenoid gear;

    public RobotADrive() {
        latest = this;
        if (pneumatics) {
            gear = new DoubleSolenoid(2, 3);
        }
        right.add(new WPI_TalonSRX(10));
        right.add(new WPI_TalonSRX(11));
        left.add(new WPI_TalonSRX(12));
        left.add(new WPI_TalonSRX(13));
        right.setEncoder(new Encoder(7, 6));
        left.setEncoder(new Encoder(8,9));
//        left.setDirection(Drivebox.DIRECTION_BACKWARD);
//        right.setDirection(Drivebox.DIRECTION_BACKWARD);
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
            log("Gear: UP");
        }
    }

    public void gearDown() {
        if (pneumatics) {
            gear.set(DoubleSolenoid.Value.kReverse);
            log("Gear: DOWN");
        }
    }

    @Override
    public void set(double speed, double turn) {
        latestX = turn;
        latestY = speed;
        super.set(speed, turn);
        log("EL: "+left.getEncoder().getRaw()+" ER: "+right.getEncoder().getRaw());
    }
}
