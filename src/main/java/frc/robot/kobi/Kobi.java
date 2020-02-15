package frc.robot.kobi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.Bot;
import frc.robot.base.control.path.PathManager;
import frc.robot.base.rgb.RGB;
import frc.robot.base.utils.auto.Autonomous;
import frc.robot.kobi.systems.KobiDrive;
import frc.robot.kobi.systems.KobiFeeder;
import frc.robot.kobi.systems.KobiShooter;

import java.awt.*;
import java.util.ArrayList;

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
        autonomous.loop();
    }

    @Override
    public void teleop() {
        set("time", String.valueOf(millis()));
        rgb.setColor(new Color(40, 40, (int) (40 * Math.abs(driver.getY()))));
        drive.driveManual(-driver.getY(), driver.getX());
    }
}