package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMax;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.base.drive.DifferentialDrive;

public class KobiDrive extends DifferentialDrive<CANSparkMax> {
    public KobiDrive() {
        left.addMotor(new CANSparkMax(50, CANSparkMaxLowLevel.MotorType.kBrushless));
        right.addMotor(new CANSparkMax(51, CANSparkMaxLowLevel.MotorType.kBrushless));
        left.setEncoder(new Encoder(0, 1));
        right.setEncoder(new Encoder(2, 3));
    }
}
