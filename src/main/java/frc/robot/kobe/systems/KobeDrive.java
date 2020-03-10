package frc.robot.kobe.systems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.utils.MotorGroup;

import java.util.ArrayList;

public class KobeDrive extends DifferentialDrive<PWMSparkMax> {

    private ArrayList<PWMSparkMax> drives = new ArrayList<>();

    public KobeDrive() {
        drives.add(left.addMotor(new PWMSparkMax(2)));
        drives.add(left.addMotor(new PWMSparkMax(3)));
        drives.add(right.addMotor(new PWMSparkMax(4)));
        drives.add(right.addMotor(new PWMSparkMax(5)));

        right.setEncoder(new Encoder(0, 1));
        left.setEncoder(new Encoder(3, 2));

        right.setDirection(MotorGroup.BACKWARD);
        resetOdometry();
    }

}
