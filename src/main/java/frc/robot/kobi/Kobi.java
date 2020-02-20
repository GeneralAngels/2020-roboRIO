package frc.robot.kobi;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.base.Bot;
import frc.robot.base.control.path.PathManager;
import frc.robot.base.rgb.RGB;
import frc.robot.base.utils.Toggle;
import frc.robot.base.utils.auto.Autonomous;
import frc.robot.kobi.systems.KobiDrive;
import frc.robot.kobi.systems.KobiFeeder;
import frc.robot.kobi.systems.KobiShooter;
import org.opencv.core.Mat;

public class Kobi extends Bot {

    /**
     * Pinout as of 16/02/2020
     * DIO:
     * 0, 1 - Left Encoder
     * 2, 3 - Right Encoder
     * 4, 5 - Min/Max LimS/W
     * 6, 7 - Turret LimS/W
     * <p>
     * PWM:
     * 0, 1 - Left Victors
     * 2, 3 - Right Victors
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
     * 50, 51, ~52~ - Left Sparks
     * 53, 54, ~55~ - Right Sparks
     * <p>
     * Talon Encoders:
     * Slide - has encoder
     * Turret - has encoder
     * Shooter - has encoder
     */

    // Joystick
    private Joystick driverLeft;
    private Joystick driverRight;
    private XboxController operator;

    // Modules
    private KobiDrive drive;
    private KobiFeeder feeder;
    private KobiShooter shooter;
    private RGB rgb;

    private Autonomous autonomous;
    private PathManager manager;

    // Auto trigger
    private Toggle autoToggle;

    // Robot mode
    private boolean isAutonomous = false;

    // PDP
    private static PowerDistributionPanel pdp = new PowerDistributionPanel(0);

    public static void setupMotor(WPI_TalonSRX talon, FeedbackDevice feedbackDevice, double kP, double kI, double kD, double kF) {
        talon.setSelectedSensorPosition(0);
        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(feedbackDevice, 0, 30);
        talon.configNominalOutputForward(0, 30);
        talon.configNominalOutputReverse(0, 30);
        talon.configPeakOutputForward(1, 30);
        talon.configPeakOutputReverse(-1, 30);
        talon.config_kP(0, kP, 30);
        talon.config_kI(0, kI, 30);
        talon.config_kD(0, kD, 30);
        talon.config_kF(0, kF, 30);
    }

    public Kobi() {
        // Joystick
        driverLeft = new Joystick(0);
        driverRight = new Joystick(2);
        operator = new XboxController(2);

        // Modules
        rgb = new RGB();
        drive = new KobiDrive();
        manager = new PathManager(drive);
        shooter = new KobiShooter(this);
        feeder = new KobiFeeder(this);
        autonomous = new Autonomous(this);
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

        // Initialize autoToggle
        autoToggle = new Toggle("auto", new Toggle.OnStateChanged() {
            @Override
            public void onStateChanged(boolean state) {
                isAutonomous = !isAutonomous;
            }
        });
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
        handleControllers();

        // Update shooter positions
        shooter.updatePositions();
    }

    private void handleControllers() {
        // Handle operator
        autoToggle.update(operator.getXButton());
        // Make sure robot is not in auto
        if (!isAutonomous) {
            // Move slider & feeder
            if (operator.getPOV() != -1) {
                // Block other roller input
                if (operator.getPOV() == 90 || operator.getPOV() == 270) {
                    // Roll in
                    feeder.roll(KobiFeeder.Direction.In);
                    // Check slider direction
                    if (operator.getPOV() == 90) {
                        feeder.slide(KobiFeeder.Direction.In);
                    } else if (operator.getPOV() == 270) {
                        feeder.slide(KobiFeeder.Direction.Out);
                    }
                } else {
                    // Stop feeder
                    feeder.slide(KobiFeeder.Direction.Stop);
                }
            } else {
                // Allow roller input
                if (operator.getBumper(GenericHID.Hand.kRight)) {
                    // Roll in
                    feeder.roll(KobiFeeder.Direction.In);
                } else if (operator.getBumper(GenericHID.Hand.kLeft)) {
                    // Roll out
                    feeder.roll(KobiFeeder.Direction.Out);
                } else {
                    // Roll stop
                    feeder.roll(KobiFeeder.Direction.Stop);
                }
                // Stop feeder
                feeder.slide(KobiFeeder.Direction.Stop);
            }
            // Feeder & shooter
            double shoot = deadband(operator.getY(GenericHID.Hand.kRight));
            // Set velocity
            shooter.setShooterVelocity(30 * shoot);
            // Set feeder
            double feed = operator.getTriggerAxis(GenericHID.Hand.kLeft) - operator.getTriggerAxis(GenericHID.Hand.kRight);
            feeder.feed(fromJoystick(feed)); // In when shooter is out
            // Turret
            double left = -deadband(operator.getX(GenericHID.Hand.kLeft));
            // Set turret speed
            shooter.setTurretVelocity(left / 5);
            // Hood position
            if (operator.getAButton()) {
                shooter.setHoodPosition(KobiShooter.HOOD_MINIMUM_ANGLE);
            } else if (operator.getBButton()) {
                shooter.setHoodPosition(45);
            } else {
                shooter.setHoodPosition(KobiShooter.HOOD_MAXIMUM_ANGLE);
            }
        }
        // Read joysticks
        double[] wheelsToRobot = drive.wheelsToRobot(-driverLeft.getY(), -driverRight.getY());
        drive.driveVector(wheelsToRobot[0], wheelsToRobot[1]);
    }

    public boolean isAutonomous() {
        return isAutonomous;
    }

    private KobiFeeder.Direction fromJoystick(double value) {
        if (Math.abs(value) < 0.05)
            return KobiFeeder.Direction.Stop;
        if (value < 0)
            return KobiFeeder.Direction.In;
        else
            return KobiFeeder.Direction.Out;
    }

    private double deadband(double value) {
        if (Math.abs(value) < 0.05) {
            return 0;
        }
        return value;
    }
}