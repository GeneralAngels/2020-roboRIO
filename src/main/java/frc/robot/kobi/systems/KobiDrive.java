package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.utils.MotorGroup;

public class KobiDrive extends DifferentialDrive<VictorSP> {
    public KobiDrive() {

        left.addMotor(new VictorSP(0));
        left.addMotor(new VictorSP(1));
        right.addMotor(new VictorSP(2));
        right.addMotor(new VictorSP(3));

        right.setEncoder(new Encoder(0, 1));
        left.setEncoder(new Encoder(3, 2));

        right.setDirection(MotorGroup.BACKWARD);
        resetOdometry();
    }


}
