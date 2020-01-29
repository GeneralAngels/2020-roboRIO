package frc.robot.shuby;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.base.Bot;

public class Shuby extends Bot {

    private ShubyDrive drive;
    private Joystick joystick;

    public Shuby() {
        drive = new ShubyDrive();
        joystick = new Joystick(0);
    }

    @Override
    public void teleop() {
        drive.setNoPID(-joystick.getY(), joystick.getX());
    }
}
