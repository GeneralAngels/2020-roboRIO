package frc.robot.thucc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.base.drive.DiffDrive;

public class ThuccDrive extends DiffDrive<CANSparkMax> {
    public ThuccDrive() {
        CANSparkMax left1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax left2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax right1 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax right2 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        left2.setInverted(false);
        right2.setInverted(false);
        left.addMotor(left1);
        left.addMotor(left2);
        right.addMotor(right1);
        right.addMotor(right2);
        left.setEncoder(new Encoder(1, 0));
        right.setEncoder(new Encoder(2, 3));
    }
}
