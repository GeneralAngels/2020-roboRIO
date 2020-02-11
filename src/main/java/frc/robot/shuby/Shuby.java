package frc.robot.shuby;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.base.Bot;
import frc.robot.base.drive.DifferentialDrive;

public class Shuby extends Bot {

    private ShubyDrive drive;
    private Joystick joystick;

    public Shuby() {
        drive = new ShubyDrive();
        joystick = new Joystick(0);
        drive.setMode(DifferentialDrive.Mode.Manual);
    }

    @Override
    public void teleop() {
        drive.driveManual(-joystick.getY(), joystick.getX());
        drive.loop();
    }
}
