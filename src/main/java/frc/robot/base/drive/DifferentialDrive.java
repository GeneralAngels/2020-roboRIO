package frc.robot.base.drive;

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

// TODO redo the whole thing
// TODO once complete, add copyright comment, its too embarrassing rn

public class DifferentialDrive<T extends SpeedController> extends Module {

    private static final double TOLERANCE = 0.05;
    private static final double DEGREE_TOLERANCE = 3;

    private static final double WHEEL_DISTANCE = 0.7112; // TODO 2020 update
    private static final double WHEEL_RADIUS = 0.0762; // TODO 2020 update

    private static final double MAX_V = 1.5;
    private static final double MAX_A = 1.5;

    private static final double TICKS_PER_REVOLUTION = 2048;
    private static final double METERS_PER_REVOLUTION = (2 * Math.PI) * (4 * 2.45);
    private static final double ENCODER_TO_RADIAN = (2 * Math.PI) / TICKS_PER_REVOLUTION;

    private double gyroscopeOffset = 0;

    // Odometry
    private double x = 0;
    private double y = 0;
    private double theta = 0;

    // Encoders
    private double[] lastEncoders = new double[2];
    private double[] currentEncoders = new double[2];

    // Errors
    private double[] lastErrors = new double[3];
    private double[] currentErrors = new double[3];

    // Modules
    public PID motorControlLeftVelocity;
    public PID motorControlRightVelocity;
    public PID motorControlLeftPosition;
    public PID motorControlRightPosition;
    public MotorGroup<T> left;
    public MotorGroup<T> right;
    public Gyroscope gyro;
    public Odometry odometry;
    // Outputs
    private double[] motorOutputs = new double[2];

    private int trajectoryIndex;

    private Mode driveMode = Mode.Disable;

    // Excuse me WTF
    public double Kv = 3;
    public double Ktheta = 1.3 * MAX_V; // Note that low values (for example 0.33) means it should get to the setpoint in ~3 seconds! // 1.6*MAX_V
    public double Kcurv = 1 * MAX_V; //0.45*MAX_V
    public double Komega = 0;
    public boolean check = true;
    public boolean check2 = true;
    public double desiredOmega;
    public double desiredVelocity;
    public double desiredVelocityPrev = 0;
    public double acc = 0;

    private double currentVoltage = 12;

    public DifferentialDrive() {
        super("drive");

        left = new MotorGroup<>("left");
        right = new MotorGroup<>("right");

        motorControlLeftVelocity = new PID("pid_left_velocity", 0, 0.03, 0, 0.24);
        motorControlRightVelocity = new PID("pid_right_velocity", 0, 0.03, 0, 0.24);
        motorControlLeftPosition = new PID("pid_left_position", 3, 0.1, 0.2, 0);
        motorControlRightPosition = new PID("pid_right_position", 3, 0.1, 0.2, 0);

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
        // Reset all
        resetTrajectory();
        initializeGyroscope(new Gyroscope());
        // Default mode
        setMode(Mode.Disable);
    }

    public void printEncoders() {
        log("left: " + left.getEncoder().getRaw() + "right: " + right.getEncoder().getRaw());
    }

    public void resetTrajectory() {
        PathManager.createPath(new ArrayList<>());
        this.trajectoryIndex = 1;
    }

    public void updateOdometry() {
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
        odometry.setTheta(theta = gyro.getAngle());
    }

    public void initializeGyroscope(Gyroscope gyro) {
        this.gyro = gyro;
        this.gyro.reset();
        this.gyroscopeOffset = this.gyro.getAngle();
    }

    public void loop() {
        if (driveMode == Mode.Disable) {
            log("Disabled! - Use .setMode!");
        } else {
            if (driveMode == Mode.Trajectory) {
                trajectoryFollow();
            }
            direct(motorOutputs[0], motorOutputs[1]);
            updateOdometry();
        }
        // todo
//        if (isAuto) {
//            Trajectory.State goal = PathManager.getTrajectory().sample(millis());
//            //log("curve: " + goal.curvatureRadPerMeter);
//            double[] speeds = calcErrors(goal, goal); // Sent goal twice because there is no meaning to prevGoal
//            if (true) {//!isDone(getPose(), trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters) || goal.velocityMetersPerSecond + (errorDistance * 0.7) > 0.05) {
//                //set(speeds[0] * 8, speeds[1] * 0.7);
//                setVector(goal.velocityMetersPerSecond + speeds[0], (goal.velocityMetersPerSecond * goal.curvatureRadPerMeter) + speeds[1]);
//                log("done");
//            } else {
//                setMode(false);
//            }
//        } else {
//        }
    }

    public void trajectoryFollow() {
        Trajectory.State startPoint = PathManager.getTrajectory().getStates().get(1);
        Trajectory.State endPoint = PathManager.getTrajectory().getStates().get(PathManager.getTrajectory().getStates().size() - 1);
//        if (check) {
//            MAX_V = 0.75 / Math.abs(Math.atan2(endPoint.poseMeters.getTranslation().getY() - startPoint.poseMeters.getTranslation().getY(), endPoint.poseMeters.getTranslation().getX() - startPoint.poseMeters.getTranslation().getX()));
//            if (MAX_V > 1.5)
//                MAX_V = 1.5;
//            MAX_A = 0.5 / Math.abs(Math.atan2(endPoint.poseMeters.getTranslation().getY() - startPoint.poseMeters.getTranslation().getY(), endPoint.poseMeters.getTranslation().getX() - startPoint.poseMeters.getTranslation().getX()));
//            if (MAX_A > 1.5)
//                MAX_A = 1.5;
//            check = false;
//        }
        log("MAX_V: " + MAX_V);
        log("MAX_A: " + MAX_A);
        double omega = gyro.getAngularVelocity();
        log("omega: " + omega);
        Trajectory.State currentGoal = PathManager.getTrajectory().getStates().get(trajectoryIndex);
        double curvature = currentGoal.curvatureRadPerMeter;
        if (trajectoryIndex < PathManager.getTrajectory().getStates().size() - 2) {
            Trajectory.State curvatureGoal = PathManager.getTrajectory().getStates().get(trajectoryIndex + 2);
            curvature = curvatureGoal.curvatureRadPerMeter;
        }
        curvature = Math.abs(curvature);
        set("curvature", "" + curvature);
        double[] errors = calculateErrors(currentGoal);

        if (trajectoryIndex < PathManager.getTrajectory().getStates().size() - 1) { //length of the trajectory
            desiredVelocity = MAX_V - curvature * Kcurv;
            acc = MAX_A - (PathManager.movingAverageCurvature(PathManager.getTrajectory().getStates()) / 2.0);
            log("acc: " + acc);
            desiredVelocity = Math.min(desiredVelocity, desiredVelocityPrev + (acc * 0.02));
            if ((desiredVelocity >= 0 && desiredVelocity < 0.5) || (MAX_V < (curvature * Kcurv))) {
                desiredVelocity = 0.5;
            }
            desiredOmega = errors[2] * Ktheta - omega * Komega;
            log("desiredVelocity: " + desiredVelocity);
            super.set("desiredVelocity", "" + desiredVelocity);
            super.set("desiredOmega", "" + desiredOmega);
            driveVector(desiredVelocity, desiredOmega);
            log("not last point");
            if (errors[0] < errors[1]) {
                trajectoryIndex++;
            }
        } else {
            toReverse(getPose(), PathManager.getTrajectory().getStates().get(PathManager.getTrajectory().getStates().size() - 1).poseMeters);
            desiredVelocity = errors[0] * Kv;
            desiredOmega = (currentGoal.poseMeters.getRotation().getRadians() - Math.toRadians(theta)) * Ktheta - omega * Komega;
            super.set("desiredVelocity", "" + desiredVelocity);
            super.set("desiredOmega", "" + desiredOmega);
            if (!isDone(getPose(), PathManager.getTrajectory().getStates().get(PathManager.getTrajectory().getStates().size() - 1).poseMeters)) {
                if (errors[0] > TOLERANCE) {
                    driveVector(desiredVelocity, desiredOmega * 1.3);
                    log("close");
                } else {
                    driveVector(0, desiredOmega * 1.8);
                    log("almost");
                }
                set("last point: ", "true");

            } else {
                driveVector(0, 0);
                log("done");
            }
        }
        if (trajectoryIndex < PathManager.getTrajectory().getStates().size() - 1 && errors[0] < errors[1]) {
            ++trajectoryIndex;
        }
        desiredVelocityPrev = desiredVelocity;
    }

    public double[] calculateErrors(Trajectory.State currentGoal) {
        // Previous error
        this.lastErrors = currentErrors;
        // Assign values
        double errorX = currentGoal.poseMeters.getTranslation().getX() - x;
        double errorY = currentGoal.poseMeters.getTranslation().getY() - y;
        // Theta calculation
        double errorTheta = (Math.atan2(errorY, errorX) - Math.toRadians(theta)) % (2 * Math.PI);
        // Return tuple
        return currentErrors = new double[]{Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)), lastErrors[1], errorTheta};
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    }

    public boolean isDone(Pose2d current, Pose2d end) {
        // Assign values
        double errorX = current.getTranslation().getX() - end.getTranslation().getX();
        double errorY = current.getTranslation().getY() - end.getTranslation().getY();
        // Calculate errors
        double errorTheta = Math.abs(current.getRotation().getDegrees() - end.getRotation().getDegrees());
        double errorPosition = errorX * Math.cos(Math.toRadians(theta)) + errorY * Math.sin(Math.toRadians(theta));
        // Check against tolerances
        return Math.abs(errorPosition) < TOLERANCE && errorTheta < DEGREE_TOLERANCE;
    }

    public void toReverse(Pose2d current, Pose2d end) {
        // Assign values
        double errorX = current.getTranslation().getX() - end.getTranslation().getX();
        double errorY = current.getTranslation().getY() - end.getTranslation().getY();
        // Calculate errors
        double errorTheta = Math.abs(current.getRotation().getDegrees() - end.getRotation().getDegrees());
        double errorPosition = errorX * Math.cos(Math.toRadians(theta)) + errorY * Math.sin(Math.toRadians(theta));
        // ToDo wHaT deFuk
        if ((errorPosition > 0 && Kv > 0) || (errorPosition < 0 && Kv < 0)) {
            if (check2) {
                Kv *= -1;
                check2 = false;
            }
        } else
            check2 = true;
    }

    // Drive output setters

    public void driveManual(double speed, double turn) {
        motorOutputs = new double[]{(turn + speed), (turn - speed)};
    }

    public void driveVector(double velocity, double omega) {
        motorOutputs = calculateOutputs(Math.abs(velocity) < TOLERANCE ? 0 : velocity, Math.abs(omega) < TOLERANCE ? 0 : omega);
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

    // Modes

    public void setMode(Mode mode) {
        this.driveMode = mode;
    }

    public enum Mode {
        Manual,
        Trajectory,
        Disable
    }
}
