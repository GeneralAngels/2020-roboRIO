package frc.robot.efrat;

import edu.wpi.first.wpilibj.*;
import frc.robot.bobot.Bobot;
import frc.robot.bobot.rgb.RGB;
import frc.robot.bobot.utils.Toggle;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Klein;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.Tomer;
import frc.robot.efrat.systems.rgb.RobotIdle;
import frc.robot.efrat.systems.robota.RobotADrive;
import frc.robot.efrat.systems.robotc.RobotCDrive;
import org.json.JSONException;
import org.json.JSONObject;

import java.awt.*;

@SuppressWarnings("ALL")
public class RobotC extends Bobot {
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
        shiriToggle = new Toggle(new Toggle.Change() {
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
