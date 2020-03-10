package frc.robot.kobe;

import com.ga2230.shleam.advanced.frc.FRCRobot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.base.control.path.PathManager;
import frc.robot.base.control.path.Point;
import frc.robot.base.rgb.RGB;
import frc.robot.base.utils.General;
import frc.robot.kobe.systems.KobeDrive;
import frc.robot.kobe.systems.KobeFeeder;
import frc.robot.kobe.systems.KobeShooter;

public class Kobe extends FRCRobot {

    /**
     * Pinout as of 16/02/2020
     * DIO:
     * 0, 1 - Left Encoder
     * 2, 3 - Right Encoder
     * 4, 5 - Min/Max LimS/W
     * 6, 7 - Turret LimS/W
     * <p>
     * PWM:
     * 2, 3 - Left Sparks
     * 4, 5 - Right Sparks
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
     * 30, 31 - Left Sparks
     * 32, 33 - Right Sparks
     * <p>
     * Talon Encoders:
     * Slide - has encoder
     * Turret - has encoder
     * Shooter - has encoder
     */

    private static final double DEADBAND = 0.05;

    private static long offset;

    // Joystick
    private Joystick driverLeft;
    private Joystick driverRight;
    private XboxController operator;

    // Modules
    private KobeDrive drive;
    private KobeFeeder feeder;
    private KobeShooter shooter;
    private RGB rgb;

    private PathManager manager;

    // PDP
    private static PowerDistributionPanel pdp = new PowerDistributionPanel(0);

    public Kobe() {

        // Controller initialization
        operator = new XboxController(0);
        driverLeft = new Joystick(1);
        driverRight = new Joystick(2);

        // Module initialization
        rgb = new RGB();
        drive = new KobeDrive();
        manager = new PathManager(drive);
        feeder = new KobeFeeder();
        shooter = new KobeShooter();

        // Adopt children
        adopt(manager);
        adopt(shooter);
        adopt(feeder);
        adopt(drive);
        adopt(rgb);

        // Register functions

        manager.createTrajectory(new Point(1, 0, 0, 0), false);

        // RGB mode
        rgb.setMode(RGB.Mode.Fill);
    }

    private void updateAll() {
        // Voltage
        drive.updateVoltage(pdp.getVoltage());

        // Time
        set("time", String.valueOf(millis() - offset));

        // Update odometry
        drive.updateOdometry();

        // Update shooter positions
        shooter.updatePositions();
    }

    @Override
    public void autonomousSetup() {
        // Update offset
        offset = millis();
    }

    @Override
    public void teleopSetup() {
        autonomousSetup();
    }

    @Override
    public void autonomousLoop() {
        // Update all
        updateAll();

        // Runtime
        autonomous.next();
    }

    @Override
    public void teleopLoop() {
        // Update all
        updateAll();
//        log("right: " + drive.right.getEncoder().getRaw());
//        log("left: " + drive.left.getEncoder().getRaw());
        // Production
        handleControllers();
//        if (operator.getBButton())
//            manager.followTrajectory(false);
//        else
//            drive.driveManual(-operator.getY(GenericHID.Hand.kRight) / 3, operator.getX(GenericHID.Hand.kRight) / 3);
    }


    private void handleControllers() {
        // Setpoint lock
        shooter.setSetPointLock(operator.getAButton());

        // Setpoints

        double shooterVelocity = 0;
        double turretVelocity = 0;
        double hoodPosition = KobeShooter.HOOD_SAFE_MAXIMUM_ANGLE;

        boolean rollerSpeed = true;

        KobeFeeder.Direction feederDirection = KobeFeeder.Direction.Stop;
        KobeFeeder.Direction rollerDirection = KobeFeeder.Direction.Stop;
        KobeFeeder.Direction sliderDirection = KobeFeeder.Direction.Stop;

        // Flywheel
        if (!operator.getAButton()) {
            // Shooter velocity from controller
            shooterVelocity = General.deadband(-operator.getY(GenericHID.Hand.kRight), DEADBAND) * 34;
        } else {
            shooterVelocity = shooter.getShooterSetPoint();
        }
        // Check flywheel acceleration to initiate feeding
        boolean flywheelAccelerated = shooter.setShooterVelocity(shooterVelocity);
        // Make sure the input is not 0 and that we accelerated
        if (flywheelAccelerated && General.deadband(shooterVelocity, DEADBAND) != 0) {
            if (shooterVelocity > 0)
                feederDirection = KobeFeeder.Direction.In;
        }
        // Read feeder delta from operator
        double feederDeltaManual = General.deadband(operator.getTriggerAxis(GenericHID.Hand.kLeft), DEADBAND) - General.deadband(operator.getTriggerAxis(GenericHID.Hand.kRight), DEADBAND);
        // Check if the delta is not 0
        if (feederDeltaManual != 0) {
            feederDirection = General.fromJoystick(feederDeltaManual, DEADBAND);
        }
        // Set feeder
        feeder.feed(feederDirection);

        // Hood
        if (!operator.getAButton()) {
            if (General.deadband(shooterVelocity, DEADBAND) != 0) {
                hoodPosition = KobeShooter.HOOD_SAFE_MINIMUM_ANGLE;
            }
        } else {
            if (shooter.getHoodSetPoint() < KobeShooter.HOOD_SAFE_MINIMUM_ANGLE) {
                hoodPosition = KobeShooter.HOOD_SAFE_MINIMUM_ANGLE;
            } else if (shooter.getHoodSetPoint() > KobeShooter.HOOD_SAFE_MAXIMUM_ANGLE) {
                hoodPosition = KobeShooter.HOOD_SAFE_MAXIMUM_ANGLE;
            } else {
                hoodPosition = shooter.getHoodSetPoint();
            }
        }
        // Set hood position
        shooter.setHoodPosition(hoodPosition);

        // Turret
        if (!operator.getXButton()) {
            // Manual control
            if (operator.getStartButton()) {
                turretVelocity += 1;
            }
            if (operator.getBackButton()) {
                turretVelocity -= 1;
            }
        } else {
            turretVelocity = shooter.getTurretSetPoint();
        }
        // Set turret
        shooter.setTurretVelocity(-turretVelocity / 5);

        // Move slider
        // Block other roller input
        if (operator.getPOV() == 0 || operator.getPOV() == 180) {
            rollerDirection = KobeFeeder.Direction.In;
            // Check slider direction
            if (operator.getPOV() == 0) {
                rollerSpeed = true;
                sliderDirection = KobeFeeder.Direction.Out;
            } else if (operator.getPOV() == 180) {
                rollerSpeed = false;
                sliderDirection = KobeFeeder.Direction.In;
            }
        }
        // Read other roller direction
        if (operator.getBumper(GenericHID.Hand.kRight)) {
            rollerDirection = KobeFeeder.Direction.In;
        } else if (operator.getBumper(GenericHID.Hand.kLeft)) {
            rollerDirection = KobeFeeder.Direction.Out;
        }
        // Slide & Roll
        feeder.roll(rollerDirection, rollerSpeed);
        feeder.slide(sliderDirection);
        // Drive
        drive.direct(-driverLeft.getY(), -driverRight.getY());
    }
}