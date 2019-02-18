package frc.robot.yompa;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.bobot.drive.DifferentialDrive;
import frc.robot.bobot.drive.Drivebox;
import frc.robot.bobot.drive.Gyroscope;
import org.json.JSONObject;

public class YompaDrive extends DifferentialDrive<VictorSP> {
    private static YompaDrive latest;

    public YompaDrive() {
        latest = this;
        right.add(new VictorSP(0));
        right.add(new VictorSP(1));
        left.add(new VictorSP(2));
        left.add(new VictorSP(3));
//        left.setEncoder(new Encoder(7, 6));
//        right.setEncoder(new Encoder(4, 5));
//        right.setDirection(Drivebox.DIRECTION_BACKWARD);
//        left.getEncoder().reset();
//        right.getEncoder().reset();
//        gyro = new Gyroscope();
//        initGyro(gyro);

    }

    public static YompaDrive getInstance() {
        return latest;
    }

    public void setBench(double speed, double turn) {
        direct(speed, turn);
    }

}
