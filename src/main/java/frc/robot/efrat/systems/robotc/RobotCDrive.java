package frc.robot.efrat.systems.robotc;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.bobot.drive.DifferentialDrive;
import frc.robot.bobot.drive.Drivebox;
import frc.robot.bobot.drive.Gyroscope;
import org.json.JSONObject;

public class RobotCDrive extends DifferentialDrive<WPI_TalonSRX> {
    private static RobotCDrive latest;

    public RobotCDrive() {
        latest = this;
        right.add(new WPI_TalonSRX(10));
        right.add(new WPI_TalonSRX(11)); //r1
        left.add(new WPI_TalonSRX(12)); // l1
        left.add(new WPI_TalonSRX(13)); // l2
        left.setDirection(Drivebox.DIRECTION_BACKWARD);
    }

    public static RobotCDrive getInstance() {
        return latest;
    }
}
