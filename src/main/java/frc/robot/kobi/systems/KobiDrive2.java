package frc.robot.kobi.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.utils.MotorGroup;

public class KobiDrive2 extends DifferentialDrive<CANSparkMax> {
    public KobiDrive2() {
        left.addMotor(new CANSparkMax(50, CANSparkMaxLowLevel.MotorType.kBrushless));
        left.addMotor(new CANSparkMax(51, CANSparkMaxLowLevel.MotorType.kBrushless));

        right.addMotor(new CANSparkMax(53, CANSparkMaxLowLevel.MotorType.kBrushless));
        right.addMotor(new CANSparkMax(54, CANSparkMaxLowLevel.MotorType.kBrushless));

        right.setEncoder(new Encoder(0, 1));
        left.setEncoder(new Encoder(2, 3));

        right.setDirection(MotorGroup.BACKWARD);
        resetOdometry();
    }


}
