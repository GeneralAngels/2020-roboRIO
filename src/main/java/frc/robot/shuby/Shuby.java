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
        joystick1 = new Joystick(0);
//        joystick2 = new Joystick(1);
    }

    @Override
    public void teleop() {
//        drive.direct(-joystick1.getY(), joystick2.getY());
        drive.driveManual(-joystick1.getY(), joystick1.getX());
    }
}
