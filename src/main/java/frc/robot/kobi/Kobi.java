package frc.robot.kobi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.base.Bot;
import frc.robot.base.rgb.RGB;
import frc.robot.base.rgb.patterns.TestPush;
import frc.robot.base.utils.Batteries;
import frc.robot.kobi.systems.KobiDrive;
import frc.robot.kobi.systems.KobiFeeder;
import frc.robot.kobi.systems.KobiShooter;
import frc.robot.base.control.path.PathManager;
import frc.robot.shuby.ShubyDrive;

import java.util.Arrays;
import java.util.Random;

public class Kobi extends Bot {

    // Joystick
    private Joystick driver;
    private Joystick driver2;
    private PathManager pather;
    // Modules
    private KobiDrive drive;
    private KobiFeeder feeder;
    private KobiShooter shooter;
    private Batteries batteries;
    private RGB rgb;
    private double time = 0;
    boolean toggle = true;
    private int trajectoryIndex = 0;
    // PDP
    private PowerDistributionPanel pdp;
    private boolean go = false;
    private boolean tog1 = false;
    private boolean tog1pressed = false;
    private boolean tog2 = false;
    private boolean tog2pressed = false;

    private double[] splinex;
    private double[] spliney;

    public Kobi() {
        // Joystick
        driver = new Joystick(0);
        driver2 = new Joystick(1);
//        driverTest =
        log(driver + " jyro thingy");
        // PDP
//        pdp = new PowerDistributionPanel(2);
        // Modules
        batteries = new Batteries();
        drive = new KobiDrive();
        rgb = new RGB(30);
        rgb.setPattern(new TestPush());
        pather = new PathManager(drive);
        // Ah yes, enslaved modules
        enslave(batteries);
        enslave(drive);
        enslave(rgb);
        drive.setTrajectory(drive.trajectory);
        drive.updateOdometry();

        splinex = pather.createSpline(0.5,1,0.5,-1);
        for (int i = 0; i < splinex.length; i++) {
            log("*"+splinex[i]);
        }
        spliney = pather.createSpline(0.5,-1,0.5,0);
        log("*"+Arrays.toString(spliney));
    }

    @Override
    public void teleop() {
//
//        log("Yeeete?");
//        set("left", String.valueOf(drive.left.getEncoder().get()));
//        set("right", String.valueOf(drive.right.getEncoder().get()));
//        set("random", String.valueOf(new Random().nextInt(100)));
        set("left_velocity", String.valueOf(drive.motorControlLeftVelocity));
        set("right_velocity", String.valueOf(drive.motorControlRightVelocity));
        set("time", String.valueOf(millis()));


//        drive.setNoPID(-driver.getY()/5, driver.getX()/5);
        //log("left: " + drive.left.getEncoder().getRaw() + "     right: " + drive.right.getEncoder().getRaw());
    }
}
//0.2 / 0.07 = 0.85