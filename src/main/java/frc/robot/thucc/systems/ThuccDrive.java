package frc.robot.thucc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.base.drive.DifferentialDrive;

public class ThuccDrive extends DifferentialDrive<CANSparkMax> {
    public ThuccDrive() {
        CANSparkMax left1 = new CANSparkMax(30, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax left2 = new CANSparkMax(31, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax right1 = new CANSparkMax(32, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax right2 = new CANSparkMax(33, CANSparkMaxLowLevel.MotorType.kBrushless);
        //left2.setInverted(true);
        left.addMotor(left1); //30
        left.addMotor(left2); //31
        right.addMotor(right1);
        right.addMotor(right2);
        left.setEncoder(new Encoder(1, 0));
        right.setEncoder(new Encoder(2, 3));
    }
}
