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
        // Create path
        manager.createPath(new ArrayList<>(), new Pose2d(1,1, Rotation2d.fromDegrees(0)));

        // Create test-autonomous program

        // TODO only for testing
        autonomous.add("b follower create 10 10");
        autonomous.add("a follower follow");
    }

    @Override
    public void autonomous() {
        autonomous.loop();
    }

    @Override
    public void teleop() {
        set("time", String.valueOf(millis()));
//        shooter.setTurretPosition(0.5);
        shooter.setTurretPosition(driver.getY());
        // manager.follow();
    }
}