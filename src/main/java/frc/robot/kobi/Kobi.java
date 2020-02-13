package frc.robot.kobi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.base.Bot;
import frc.robot.base.control.path.PathManager;
import frc.robot.base.rgb.RGB;
import frc.robot.base.utils.auto.Autonomous;
import frc.robot.kobi.systems.KobiDrive;
import frc.robot.kobi.systems.KobiFeeder;
import frc.robot.kobi.systems.KobiShooter;

public class Kobi extends Bot {

    // Joystick
    private Joystick driver;

    // Modules
    private KobiDrive drive;
    private KobiFeeder feeder;
    private KobiShooter shooter;
    private RGB rgb;

    private Autonomous autonomous;
    private PathManager manager;

    // PDP
    private PowerDistributionPanel pdp;

    public Kobi() {

        // Joystick
        driver = new Joystick(0);

        // Modules
        drive = new KobiDrive();
//        shooter = new KobiShooter();
        manager = new PathManager(drive);
        autonomous = new Autonomous(this);

        rgb = new RGB();

        // Ah yes, enslaved modules
        enslave(autonomous);
        enslave(manager);
        enslave(shooter);
        enslave(drive);
        enslave(rgb);

        // Resets
        drive.updateOdometry();

        // RGB mode
        rgb.setMode(RGB.Mode.Slide);
    }

    @Override
    public void autonomous() {
        autonomous.loop();
    }

    @Override
    public void teleop() {
//        set("left_velocity", String.valueOf(drive.motorControlLeftVelocity));
//        set("right_velocity", String.valueOf(drive.motorControlRightVelocity));
        set("time", String.valueOf(millis()));
//        log("Enc " + shooter.getPosition());
//        shooter.setVelocity(6);
//        log("shooter encoder: " + shooter.getPosition());
//        shooter.turn(0.1);
//        shooter.setVelocity((1000.0 / 60.0) * driver.getY());
        shooter.setHoodPosition(driver.getThrottle());
//        drive.driveManual(-driver.getY() / 5, driver.getX() / 5);
//        drive
//        drive.loop();
    }
}