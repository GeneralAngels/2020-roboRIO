package frc.robot.kobi;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.base.Bot;
import frc.robot.base.rgb.RGB;
import frc.robot.base.rgb.patterns.TestPush;
import frc.robot.kobi.systems.KobiDrive;
import frc.robot.kobi.systems.KobiFeeder;
import frc.robot.kobi.systems.KobiShooter;

import java.util.Random;

public class Kobi extends Bot {

    private Joystick driver;
    private KobiDrive drive;
    private KobiFeeder feeder;
    private KobiShooter shooter;
    private RGB rgb;

    public Kobi() {
        driver = new Joystick(0);
        drive = new KobiDrive();
        rgb = new RGB(30);
        rgb.setPattern(new TestPush());
        enslave(drive);
        enslave(rgb);
    }

    @Override
    public void teleop() {
        log("Left: " + drive.left.getEncoder().get());
        log("Right: " + drive.right.getEncoder().get());
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
    }
}