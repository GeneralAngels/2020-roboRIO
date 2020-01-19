package frc.robot.kobi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.base.Bot;
import frc.robot.base.rgb.RGB;
import frc.robot.base.rgb.patterns.TestPush;
import frc.robot.base.utils.Batteries;
import frc.robot.kobi.systems.KobiDrive;
import frc.robot.kobi.systems.KobiFeeder;
import frc.robot.kobi.systems.KobiShooter;

import java.util.Random;

public class Kobi extends Bot {

    // Joystick
    private Joystick driver;
    // Modules
    private KobiDrive drive;
    private KobiFeeder feeder;
    private KobiShooter shooter;
    private Batteries batteries;
    private RGB rgb;
    private double time = 0;
    // PDP
    private PowerDistributionPanel pdp;

    public Kobi() {
        // Joystick
        driver = new Joystick(0);
        log(driver + " jyro thingy");
        // PDP
//        pdp = new PowerDistributionPanel(2);
        // Modules
        batteries = new Batteries();
        drive = new KobiDrive();
        rgb = new RGB(30);
        rgb.setPattern(new TestPush());
        // Ah yes, enslaved modules
        enslave(batteries);
        enslave(drive);
        enslave(rgb);
        drive.setTrajectory(drive.trajectory);
        drive.updateOdometry();
    }

    @Override
    public void teleop() {
//

        set("left", String.valueOf(drive.left.getEncoder().get()));
        set("right", String.valueOf(drive.right.getEncoder().get()));
        set("random", String.valueOf(new Random().nextInt(100)));
        set("time", String.valueOf(millis()));
//        drive.left.applyPower(driver.getY()*10);
//        if(Math.abs(driver.getY())<0.1){
//            drive.right.applyPower(0);
//        }
//        else {
        // drive.left.applyPower(driver.getY());
        //drive.right.applyPower(driver.getY());
//        }
//        drive.setNoPID(driver.getY(), driver.getX());
//        super.teleop();
//        batteries.updateRobot(pdp)
        //double divider = driver.getRawButton(2) ? 1 : 2;
//        log("divider");
        //drive.setNoPID(-driver.getY() / divider, driver.getX() / divider);
//        drive.loop(0);
//        drive.direct(driver.getY() + driver.getX(), driver.getY() - driver.getX());
  //      drive.updateOdometry();
        drive.updateOdometry();
//        drive.set(0.2,0);
        if (time < 10000000){
            //drive.loop(time);
            time += 0.02;
//            log("x: " + drive.x);
//            log("y:" + drive.y);
//            log("theta: " + drive.theta);
        }
//        if (time > 5){
//            log("left: " + drive.left.getEncoder().getRaw());
//            log("right: " + drive.right.getEncoder().getRaw());
//    }
        //log("ang:" + drive.theta);

    }
}