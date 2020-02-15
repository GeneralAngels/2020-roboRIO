package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.drive.DifferentialDrive;

public class KobiDrive extends DifferentialDrive<CANSparkMax> {
    public KobiDrive() {
        CANSparkMax leftMotor = new CANSparkMax(50, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax rightMotor = new CANSparkMax(51, CANSparkMaxLowLevel.MotorType.kBrushless);

//        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        left.addMotor(leftMotor);
        right.addMotor(rightMotor);

        left.setEncoder(new Encoder(3, 2));
        right.setEncoder(new Encoder(0, 1));


        resetOdometry();
        updateOdometry();
    }


}
