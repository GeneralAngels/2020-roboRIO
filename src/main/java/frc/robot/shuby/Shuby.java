package frc.robot.shuby;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.base.Bot;
import frc.robot.kobi.systems.KobiShooter;

public class Shuby extends Bot {

    private ShubyDrive drive;
    private Joystick joystick1, joystick2;

    private KobiShooter shooter;

    public Shuby() {
        drive = new ShubyDrive();
        shooter = new KobiShooter();
        joystick1 = new Joystick(1);
//        joystick2 = new Joystick(1);
    }

    @Override
    public void teleop() {
//        drive.driveManual(joystick1.getY()*(-joystick1.getTwist() + 1) / 2, joystick1.getX()*(-joystick1.getTwist() + 1)/2);
        shooter.setHoodPosition(joystick1.getY()*180);
        //  drive.driveManual(-joystick.getY(), joystick.getX());
        //  drive.loop();
    }
}
