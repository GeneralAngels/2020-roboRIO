package frc.robot.bosmat;

import edu.wpi.first.wpilibj.*;
import frc.robot.base.Bot;
import frc.robot.base.utils.Toggle;
import frc.robot.bosmat.systems.robotc.RobotCDrive;

@SuppressWarnings("ALL")
public class RobotC extends Bot {
    private Joystick driver;
    private RobotCDrive drive;
    // Toggles
    private Toggle shiriToggle;
    // Solenoid
    private DoubleSolenoid hatch;

    @Override
    public void init() {
        initDriver();
        initSystems();
        initTriggers();
    }

    private void instructions() {
        log("=======================================================");
        log("Efrat C");
        log("To Begin, Press Enable");
        log("=======================================================");
    }

    private void initDriver() {
        driver = new Joystick(0);
    }

    private void initSystems() {
        drive = new RobotCDrive();
        hatch = new DoubleSolenoid(0, 4, 7);
        addToJSON(drive);
    }

    @Override
    public void autonomous() {
        teleop();
    }

    private void initTriggers() {
        shiriToggle = new Toggle(new Toggle.OnStateChanged() {
            @Override
            public void change(boolean toggle) {
                if (toggle) {
                    hatch.set(DoubleSolenoid.Value.kForward);
                } else {
                    hatch.set(DoubleSolenoid.Value.kReverse);
                }
            }
        });
    }

    private void updateTriggers() {
        shiriToggle.update(driver.getTrigger());
    }

    @Override
    public void teleop() {
        updateTriggers();
        drive.setStickNoPID(driver.getY() / 3, driver.getX() / 2);
//        super.teleop();
    }
}
