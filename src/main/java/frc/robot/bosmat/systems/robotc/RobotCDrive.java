package frc.robot.bosmat.systems.robotc;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.utils.MotorGroup;

/**
 * What's this?
 * Drive class for Bosmat (2230 2019 Robot)
 */

public class RobotCDrive extends DifferentialDrive<WPI_TalonSRX> {
    private static RobotCDrive latest;

    public RobotCDrive() {
        latest = this;
        right.addMotor(new WPI_TalonSRX(10));
        right.addMotor(new WPI_TalonSRX(11)); //r1
        left.addMotor(new WPI_TalonSRX(12)); // l1
        left.addMotor(new WPI_TalonSRX(13)); // l2
        left.setDirection(MotorGroup.BACKWARD);
        left.setEncoder(new Encoder(1,0));
        right.setEncoder(new Encoder(2,3));
    }

    public static RobotCDrive getInstance() {
        return latest;
    }
}
