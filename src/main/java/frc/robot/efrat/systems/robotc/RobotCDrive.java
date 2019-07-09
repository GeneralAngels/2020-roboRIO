package frc.robot.efrat.systems.robotc;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.drive.MotorGroup;

public class RobotCDrive extends DifferentialDrive<WPI_TalonSRX> {
    private static RobotCDrive latest;

    public RobotCDrive() {
        latest = this;
        right.addMotor(new WPI_TalonSRX(10));
        right.addMotor(new WPI_TalonSRX(11)); //r1
        left.addMotor(new WPI_TalonSRX(12)); // l1
        left.addMotor(new WPI_TalonSRX(13)); // l2
        left.setDirection(MotorGroup.BACKWARD);
    }

    public static RobotCDrive getInstance() {
        return latest;
    }
}
