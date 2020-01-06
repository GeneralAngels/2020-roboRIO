package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.base.drive.DifferentialDrive;

public class KobiDrive extends DifferentialDrive<WPI_TalonSRX> {
    public KobiDrive() {
        left.addMotor(new WPI_TalonSRX(1));
        left.addMotor(new WPI_TalonSRX(2));
        right.addMotor(new WPI_TalonSRX(3));
        right.addMotor(new WPI_TalonSRX(4));
        left.setEncoder(new Encoder(0, 1));
        right.setEncoder(new Encoder(2, 3));
    }
}
