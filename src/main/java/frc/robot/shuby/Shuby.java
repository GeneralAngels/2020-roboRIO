package frc.robot.shuby;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.base.Bot;
import frc.robot.base.drive.DifferentialDrive;

public class Shuby extends Bot {

    private ShubyDrive drive;
    private Joystick joystick1, joystick2;

    public Shuby() {
        drive = new ShubyDrive();
        joystick1 = new Joystick(0);
        joystick2 = new Joystick(1);
        drive.setMode(DifferentialDrive.Mode.Manual);
    }

    @Override
    public void teleop() {
        drive.direct(-joystick1.getY()/2, joystick2.getY()/2);

        //  drive.driveManual(-joystick.getY(), joystick.getX());
        //  drive.loop();
    }
}
