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
    private Joystick driver;

    public Shuby() {
        drive = new ShubyDrive();
        driver = new Joystick(0);
        xbox = new XboxController(1);
        ahhhh = new CANSparkMax(54, CANSparkMaxLowLevel.MotorType.kBrushless);
        enslave(drive);
//        joystick1 = new Joystick(0);
//        joystick2 = new Joystick(1);
    }

    @Override
    public void teleop() {
        double valueY = -driver.getY();
        double valueX = driver.getX();
        if(Math.abs(valueY) < 0.05)
            valueY = 0;
        if(Math.abs(valueX) < 0.05)
            valueX = 0;
        if(driver.getRawButton(10))
            drive.driveManual(valueY, valueX);
        else if(driver.getRawButton(11))
            drive.driveVector(0.2, 0);
        else
            drive.driveManual(0,0);
//        drive.direct(-joystick1.getY(), -joystick2.getY());
//        drive.driveManual(-joystick1.getY(), joystick1.getX());
//        double divider = 1;
//        if (xbox.getAButton()) {
//            divider = 1;
//        } else {
//            divider = 2;
//        }
//        drive.driveManual(-xbox.getY(GenericHID.Hand.kRight)/divider, xbox.getX(GenericHID.Hand.kRight)/divider);
        //ahhhh.set(xbox.getY(GenericHID.Hand.kLeft));
    }
}
