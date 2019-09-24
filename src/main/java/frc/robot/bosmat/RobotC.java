package frc.robot.bosmat;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
import frc.robot.base.Bot;
import frc.robot.base.drive.Gyroscope;
import frc.robot.base.rgb.RGB;
import frc.robot.base.rgb.patterns.Rainbow;
import frc.robot.base.utils.Toggle;
import frc.robot.bosmat.systems.robotc.RobotCDrive;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

/**
 * What's this?
 * Bot class for Bosmat (2230 2019 Robot)
 */

public class RobotC extends Bot {
    private Joystick driver;
    private RobotCDrive drive;
    // Toggles
    private Toggle shiriToggle;
    private Toggle compressorToggle;
    // Solenoid
    private DoubleSolenoid hatch;
    // Shiri
    private WPI_TalonSRX motor;

    private Gyroscope gyro;
    // RGB
    private RGB rgb;

    private PowerDistributionPanel pdp;

    private Compressor compressor;

    private DriverStation ds = DriverStation.getInstance();

    @Override
    public void init() {
        super.init();
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
        compressor = new Compressor(0);
        compressor.stop();
//        rgb = new RGB(38);
//        rgb.setPattern(new Rainbow());
//        pdp=new PowerDistributionPanel();
        hatch = new DoubleSolenoid(0, 4, 7);
        motor = new WPI_TalonSRX(14);
        gyro = new Gyroscope();
        drive.gyro = gyro;
//        addToJSON(drive);
        register(drive);
    }

    @Override
    public void autonomous() {
        teleop();
    }

    private void initTriggers() {
        shiriToggle = new Toggle(state -> {
            if (state) {
                hatch.set(DoubleSolenoid.Value.kForward);
            } else {
                hatch.set(DoubleSolenoid.Value.kReverse);
            }
        });
        compressorToggle = new Toggle(state -> {
            if (state) {
                compressor.start();
            } else {
                compressor.stop();
            }
        });
    }

    private void updateTriggers() {
//        if (driver.getTrigger()) hatch.set(hatch.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        shiriToggle.update(driver.getTrigger());
        compressorToggle.update(driver.getRawButton(6));
    }

    @Override
    public void teleop() {
        log("time", millis());
        updateTriggers();
//        drive.battery = pdp.getVoltage();
        drive.battery = ds.getBatteryVoltage();
        if (driver.getRawButton(2)) {
            drive.angle_pid(1.57);
        } else {
            drive.checkAnglePID = true;
            if (driver.getRawButton(10)) {
                drive.driveStraightPID();
            } else {
                drive.setStickNoPID2(driver.getY(), driver.getX());
            }
        }

//        log("Nigger");
//        log("left encoder "+drive.left.getEncoder().getRaw());
//        log("right encoder "+drive.right.getEncoder().getRaw());
//        log("Nigger: "+gyro.getYaw()+" Nibber: "+gyro.getRoll()+" Kneegrow: "+gyro.getPitch());
//        motor.set(driver.getRawButton(4) ? 0.2 : driver.getRawButton(3) ? -0.2 : 0);
        super.teleop();
    }
}
