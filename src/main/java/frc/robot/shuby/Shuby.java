package frc.robot.shuby;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.base.Bot;
import frc.robot.kobi.systems.KobiShooter;

public class Shuby extends Bot {

    private ShubyDrive drive;
//    private Joystick joystick1, joystick2;
    private CANSparkMax ahhhh;

    private XboxController xbox;

    public Shuby() {
//        drive = new ShubyDrive();
        xbox = new XboxController(0);
        ahhhh = new CANSparkMax(54, CANSparkMaxLowLevel.MotorType.kBrushless);
//        joystick1 = new Joystick(0);
//        joystick2 = new Joystick(1);
    }

    @Override
    public void teleop() {
//        drive.direct(-joystick1.getY(), -joystick2.getY());
//        drive.driveManual(-joystick1.getY(), joystick1.getX());
//        double divider = 1;
//        if (xbox.getAButton()) {
//            divider = 1;
//        } else {
//            divider = 2;
//        }
//        drive.driveManual(-xbox.getY(GenericHID.Hand.kRight)/divider, xbox.getX(GenericHID.Hand.kRight)/divider);
        ahhhh.set(xbox.getY(GenericHID.Hand.kLeft));
    }
}
