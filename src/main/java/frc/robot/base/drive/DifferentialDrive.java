package frc.robot.base.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.Module;
import frc.robot.base.control.PID;
import frc.robot.base.control.path.PathManager;
import frc.robot.base.utils.MotorGroup;

import java.util.ArrayList;

import static java.lang.Thread.sleep;

public class DifferentialDrive<T extends SpeedController> extends Module {

    private static final double TOLERANCE = 0.05;

    private static final double WHEEL_DISTANCE = 0.7112; // TODO 2020 update
    private static final double WHEEL_RADIUS = 0.0762; // TODO 2020 update

    private static final double TICKS_PER_REVOLUTION = 2048;
    private static final double METERS_PER_REVOLUTION = (2 * Math.PI) * (4 * 2.45);
    private static final double ENCODER_TO_RADIAN = (2 * Math.PI) / TICKS_PER_REVOLUTION;

    private double gyroscopeOffset = 0;

    // Odometry
    private double x = 0;
    private double y = 0;
    private double theta = 0;
    private double omega = 0;

    // Encoders
    private double[] lastEncoders = new double[2];
    private double[] currentEncoders = new double[2];

    // Modules
    public PID motorControlLeftVelocity;
    public PID motorControlRightVelocity;
    public PID motorControlLeftPosition;
    public PID motorControlRightPosition;
    public MotorGroup<T> left;
    public MotorGroup<T> right;
    public Odometry odometry;

    private double currentVoltage = 12;

    public DifferentialDrive() {
        super("drive");
        left = new MotorGroup<>("left");
        right = new MotorGroup<>("right");

        motorControlLeftVelocity = new PID("pid_left_velocity", 0, 0.03, 0, 0.24);
        motorControlRightVelocity = new PID("pid_right_velocity", 0, 0.03, 0, 0.24);
        motorControlLeftPosition = new PID("pid_left_position", 3, 0.1, 0.2, 0);
        motorControlRightPosition = new PID("pid_right_position", 3, 0.1, 0.2, 0);

        odometry = new Odometry();

        // MotorGroups
        enslave(left);
        enslave(right);
        // Odometry
        enslave(odometry);
        // PIDs
        enslave(motorControlLeftVelocity);
        enslave(motorControlRightVelocity);
        enslave(motorControlLeftPosition);
        enslave(motorControlRightPosition);
    }

    public void printEncoders() {
        log("left: " + left.getEncoder().getRaw() + "right: " + right.getEncoder().getRaw());
    }

    public void updateOdometry() {
        if (left.hasEncoder() && right.hasEncoder()) {
            // Set lasts
            lastEncoders = currentEncoders;
            // Set currents
            currentEncoders = new double[]{left.getEncoder().getRaw(), right.getEncoder().getRaw()};
            // Calculate meters
            double leftMeters = ((currentEncoders[0] - lastEncoders[0]) / TICKS_PER_REVOLUTION) * METERS_PER_REVOLUTION;
            double rightMeters = ((currentEncoders[1] - lastEncoders[1]) / TICKS_PER_REVOLUTION) * METERS_PER_REVOLUTION;
            // Calculate distance
            double distanceFromEncoders = (leftMeters + rightMeters);
            // Divide
            distanceFromEncoders /= 2;

            // Set odometry
            odometry.setX(x += distanceFromEncoders * Math.cos(Math.toRadians(theta)));
            odometry.setY(y += distanceFromEncoders * Math.sin(Math.toRadians(theta)));
        }

        odometry.setTheta(theta = Gyroscope.getAngle());
        odometry.setOmega(omega = Gyroscope.getAngularVelocity());

    }

    public Odometry getOdometry() {
        return odometry;
    }

    // Drive output setters

    public void driveManual(double speed, double turn) {
        direct((turn + speed), (turn - speed));
    }

    public void driveVector(double velocity, double omega) {
        // Outputs
        double[] motorOutputs = calculateOutputs(Math.abs(velocity) < TOLERANCE ? 0 : velocity, Math.abs(omega) < TOLERANCE ? 0 : omega);
        direct(motorOutputs[0], motorOutputs[1]);
    }

    // Output calculations

    public double[] calculateOutputs(double speed, double turn) {
        double[] wheelSetPoints = robotToWheels(speed, turn);
        // Calculate
        double motorOutputLeft = motorControlLeftVelocity.PIDVelocity(left.getEncoder().getRaw() * ENCODER_TO_RADIAN, wheelSetPoints[0]);
        double motorOutputRight = motorControlRightVelocity.PIDVelocity(right.getEncoder().getRaw() * ENCODER_TO_RADIAN, wheelSetPoints[1]);
        // Divide
        motorOutputLeft /= currentVoltage;
        motorOutputRight /= currentVoltage;
        // Return tuple
        return new double[]{motorOutputLeft, motorOutputRight};
    }

    // Conversions

    private double[] robotToWheels(double linear, double angular) {
        // Assign
        double left = (linear / WHEEL_RADIUS) - (angular * WHEEL_DISTANCE) / (2 * WHEEL_RADIUS);
        double right = (linear / WHEEL_RADIUS) + (angular * WHEEL_DISTANCE) / (2 * WHEEL_RADIUS);
        // Return tuple
        return new double[]{left, right};
    }

    public double[] wheelsToRobot(double left, double right) {
        // Assign
        double linear = (right + left) * WHEEL_RADIUS / 2.0;
        double angular = (right - left) * WHEEL_RADIUS / WHEEL_DISTANCE;
        // Return tuple
        return new double[]{linear, angular};
    }

    // Robot outputs

    public void direct(double leftSpeed, double rightSpeed) {
        left.applyPower(leftSpeed);
        right.applyPower(rightSpeed);
    }
}
