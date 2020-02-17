package frc.robot.kobi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.base.Bot;
import frc.robot.base.control.path.PathManager;
import frc.robot.base.rgb.RGB;
import frc.robot.base.utils.auto.Autonomous;
import frc.robot.kobi.systems.KobiDrive;
import frc.robot.kobi.systems.KobiFeeder;
import frc.robot.kobi.systems.KobiShooter;

import java.awt.*;

public class Kobi extends Bot {

    /**
     * Pinout as of 16/02/2020
     * DIO:
     * 0, 1 - Left Encoder
     * 2, 3 - Right Encoder
     * 4, 5 - Maximum Slide LimS/W
     * 6, 7 - Minimum Slide LimS/W
     * 8, 9 - Turret LimS/W
     * <p>
     * PWM:
     * 0, 1, 2 - Left Victors
     * 3, 4, 5 - Right Victors
     * 6 - Hood Servo
     * <p>
     * AIO:
     * 0 - Hood Potentiometer
     * <p>
     * CAN:
     * Shooter 1,2,3 - 20, 21, 22
     * Turret - 19
     * Feeder - 18
     * Roller - 17
     * <p>
     * Talon Encoders:
     * Slide - has encoder
     * Turret - has encoder
     * Shooter - has encoder
     */

    // Joystick
    private Joystick driver;
    private XboxController xbox;

    // Modules
    private KobiDrive drive;
    private KobiFeeder feeder;
    private KobiShooter shooter;
    private RGB rgb;

    private Autonomous autonomous;
    private PathManager manager;

    // PDP
    private static PowerDistributionPanel pdp;

    public Kobi() {

        // Joystick
//        driver = new Joystick(0);
        xbox = new XboxController(0);

        // Init pdp
        pdp = new PowerDistributionPanel(0);

        // Modules
        drive = new KobiDrive();
        shooter = new KobiShooter();
        feeder = new KobiFeeder();
        manager = new PathManager(drive);
        autonomous = new Autonomous(this);
        rgb = new RGB();
        // Ah yes, enslaved modules
        enslave(autonomous);
        enslave(manager);
        enslave(shooter);
        enslave(feeder);
        enslave(drive);
        enslave(rgb);
        // Resets
        drive.updateOdometry();
        // RGB mode
        rgb.setMode(RGB.Mode.Fill);
        // Create path
//        manager.createPath(new ArrayList<>(), new Pose2d(1, 1, Rotation2d.fromDegrees(0)));

        // Create test-autonomous program
    }

    @Override
    public void autonomous() {
        // Voltage
        drive.updateVoltage(pdp.getVoltage());

        // Auto
        autonomous.loop();
    }

    @Override
    public void teleop() {
        // Time
        set("time", String.valueOf(millis()));

        // Voltage
        drive.updateVoltage(pdp.getVoltage());
        // Shit
//        rgb.setColor(new Color(60, 20, (int) (40 * Math.abs(driver.getY()))));
        double value = -xbox.getY(GenericHID.Hand.kRight);
//        if (xbox.getAButton()){
//            feeder.feedIn();
//            shooter.setShooterVelocity(value);
//        }else{
//            feeder.feedStop();
//        }
//        shooter.setHoodPosition(35);
        if (xbox.getBButton()) {
            shooter.setTurretPosition(10);
//            shooter.slide(fromJoystick(value));
        }
        if (xbox.getYButton()){
            shooter.resetTurretPosition();
        }
        if (xbox.getXButton()) {
            log("Shooter " + shooter.setHoodPosition(55));
        }
        shooter.updatePositions();
    }

    private KobiFeeder.Direction fromJoystick(double value) {
        if (Math.abs(value) < 0.05)
            return KobiFeeder.Direction.Stop;
        if (value < 0)
            return KobiFeeder.Direction.In;
        else
            return KobiFeeder.Direction.Out;
    }
}