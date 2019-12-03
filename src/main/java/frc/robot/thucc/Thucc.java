package frc.robot.thucc;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.base.Bot;
import frc.robot.thucc.systems.ThuccDrive;

public class Thucc extends Bot {

    private Joystick driver;
    private ThuccDrive drive;

    public Thucc() {
        driver = new Joystick(0);
        drive = new ThuccDrive();
    }

    @Override
    public void teleop() {
        drive.setNoPID(driver.getY(), driver.getX());
//        super.teleop();
    }
}
