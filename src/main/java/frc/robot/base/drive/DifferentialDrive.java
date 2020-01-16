package frc.robot.base.drive;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.Module;
import frc.robot.base.control.PID;
import frc.robot.base.control.path.PathFollower;
import frc.robot.base.utils.MotorGroup;

import static java.lang.Thread.sleep;

// TODO redo the whole thing
// TODO once complete, add copyright comment, its too embarrassing rn

public class DifferentialDrive<T extends SpeedController> extends Module {
    public double LSetpoint = 0;
    public double RSetpoint = 0;
    public PID motorControlLeftVelocity;
    public PID motorControlRightVelocity;
    public PID motorControlLeftPosition;
    public PID motorControlRightPosition;
    public Gyroscope gyro;
    public double WHEEL_DISTANCE = 0.6;
    public double WHEEL_RADIUS = 0.0762;
    public double MAX_V = 2;
    public double MAX_OMEGA = 3.14 * 2;
    public double gearRatio = 1.0;
    public double ENCODER_COUNT_PER_REVOLUTION = 512;
    public double ENCODER_TO_RADIAN = (Math.PI * 2) / (4 * ENCODER_COUNT_PER_REVOLUTION);
    public double setpointVPrevious = 0;
    public double setpointOmegaPrevious = 0;
    public double thetaRobotPrev = 0;
    public double x = 0;
    public double y = 0;
    public double MAX_WHEEL_VELOCITY = 4/WHEEL_RADIUS;
    public double leftMeters;
    public double rightMeters;
    public double[] VOmegaReal = {0, 0};
    public double[] VOmegaSetpoints = {0, 0};
    public double[] VSetpoints = {0, 0};
    public double[] encoders = {0, 0};
    public double[] encodersPrev = {0, 0};
    public double offsetGyro = 0;
    public double distanceFromEncoders = 0;
    public double theta = 0;
    public MotorGroup<T> left = new MotorGroup<>("left"), right = new MotorGroup<>("right");
    public Odometry odometry = new Odometry();
    public PathFollower follower;
    public Trajectory trajectory;
    public double battery = 12;
    public double batteryPrev = 0;
    public boolean checkGyro = true;

    private boolean isAuto = false;

    private double[] motorOutputs = new double[2];
    private double[] motorOutputsPrev = new double[2];

    private Trajectory.State basePose = null;

    private long trajectoryStart = 0;

    public DifferentialDrive() {
        super("drive");
        motorControlLeftVelocity = new PID("pid_left_velocity", 0, 0, 0, 0.2);
        motorControlRightVelocity = new PID("pid_right_velocity", 0, 0, 0, 0.2);
        motorControlLeftPosition = new PID("pid_left_position", 3, 0.1, 0.2, 0);
        motorControlRightPosition = new PID("pid_right_position", 3, 0.1, 0.2, 0);
        follower = new PathFollower(this);
        enslave(follower);
        enslave(left);
        enslave(right);
        enslave(odometry);
        enslave(motorControlLeftVelocity);
        enslave(motorControlRightVelocity);
        enslave(motorControlLeftPosition);
        enslave(motorControlRightPosition);
        this.trajectory = follower.createPath();
        gyro = new Gyroscope();
        initGyro(gyro);
        setMode(true);
    }

    public void setTrajectory(Trajectory trajectory) {
        this.basePose = new Trajectory.State();
        this.trajectory = trajectory;
        this.trajectoryStart = millis();
        //log("traj:" + this.trajectory);
    }

    public void setMode(boolean trajectory) {
        if (trajectory)
            this.trajectoryStart = millis();
        this.isAuto = trajectory;
    }

    public void loop(double time) {
        if (isAuto) {
            //log(time + "");
            Trajectory.State goal = this.trajectory.sample(time);
            //goal.timeSeconds = time / 1000.0;
            //goal = this.trajectory.sample(goal.timeSeconds);
            log("goal: " + goal.velocityMetersPerSecond);
            RamseteController controller = new RamseteController();
            Pose2d currentPose = new Pose2d(this.x, this.y, Rotation2d.fromDegrees(this.theta)); //x, y, rotation

            DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7112); //parameter is the wheel to wheel distance

            ChassisSpeeds adjustedSpeeds = controller.calculate(currentPose, goal);

            DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);

            double leftVelocity = wheelSpeeds.leftMetersPerSecond*10;
            double rightVelocity = wheelSpeeds.rightMetersPerSecond*10;

            //log("left Encoder: "+left.getEncoder().getRaw());
            double vel, omega;
            double[] speeds = wheelsToRobot(leftVelocity, rightVelocity);
            log("left: " + leftVelocity + "right:"  + rightVelocity);
            vel = speeds[0];
            omega = speeds[1];
            log("vel: " + vel + ",   omega: " + omega);
            set(vel, omega);
        } else {
            direct(motorOutputs[0], motorOutputs[1]);
        }
        updateOdometry();
    }



    public void setNoPID(double speed, double turn) {
        motorOutputs[0] = (turn + speed) / 2;
        motorOutputs[1] = (turn - speed) / 2;
    }

    public static double noPIDCalculateRight(double speed, double turn) {
        return (speed + turn);
    }

    public static double noPIDCalculateLeft(double speed, double turn) {
        return (speed - turn);
    }

    public void directControl(double speed, double turn) {
        double l, r;
        l = noPIDCalculateLeft(speed, turn);
        r = noPIDCalculateRight(speed, turn);
        direct(l / 1.5, r / 1.5);
        // updateOdometry();
    }

    public double[] calculateGoalPosition(double distanceCameraToGoal, double angleCameraToGoal) {
        //c - camera
        //g - goal
        double Xc, Yc, Xg, Yg;
        double angleCenterToCamera = 0;
        double distanseCenterToCamera = 0.22317;
        Xc = x - distanseCenterToCamera * Math.sin(toRadians(angleCenterToCamera + theta));
        Yc = y - distanseCenterToCamera * Math.cos(toRadians(angleCenterToCamera + theta));

        Xg = Xc + distanceCameraToGoal * Math.sin(toRadians(angleCameraToGoal + theta));
        Yg = Yc + distanceCameraToGoal * Math.cos(toRadians(angleCameraToGoal + theta));
        double[] res = {Xg, Yg};
        return res;
    }

//    @Deprecated
    public void setTank(double Vl, double Vr) {
        if (Math.abs(Vl) < 0.1)
            Vl = 0;
        if (Math.abs(Vr) < 0.1)
            Vr = 0;
        VSetpoints[0] = Vl; // * MAX_WHEEL_VELOCITY;
        VSetpoints[1] = Vr; // * MAX_WHEEL_VELOCITY;
        VOmegaSetpoints = wheelsToRobot(VSetpoints[0], VSetpoints[1]);
        motorOutputs = calculateOutputs(VOmegaSetpoints[0], VOmegaSetpoints[1]);
        battery = 12;
        battery = (0.5 * battery) + (0.5 * batteryPrev);
        if (battery > 12.0)
            battery = 12.0;
        batteryPrev = battery;
        motorOutputsPrev[0] = motorOutputs[0];
        motorOutputsPrev[1] = motorOutputs[1];
        direct(motorOutputs[0] / (battery), -motorOutputs[1] / (battery));
        updateOdometry();
    }

    public boolean checkAnglePID = true;
    public double initialAngle = 0.0;

    public void angle_pid(double target) {

        double angle = gyro.getAngle();
        if (checkAnglePID) {
//            // TODO do better then this reset
            initialAngle = angle;
            checkAnglePID = false;
        }
        setTank((-2 - motorControlLeftPosition.PIDPosition(angle, initialAngle + target)), (2 + motorControlRightPosition.PIDPosition(angle, initialAngle + target)));
    }

    public void driveStraightPID() {
        int initialSpeed = 4;
        setTank(initialSpeed + motorControlLeftPosition.PIDPosition(toRadians(gyro.getYaw()), 0), initialSpeed - motorControlRightPosition.PIDPosition(toRadians(gyro.getYaw()), 0));
    }

    public void set(double setpointV, double setpointOmega) {
        if (Math.abs(setpointV) < 0.2)
            setpointV = 0;
        if (Math.abs(setpointOmega) < 0.2)
            setpointOmega = 0;
        motorOutputsPrev[0] = motorOutputs[0];
        motorOutputsPrev[1] = motorOutputs[1];
        motorOutputs = calculateOutputs(setpointV, setpointOmega);
        setpointVPrevious = setpointV;
        setpointOmegaPrevious = setpointOmega;
        direct(motorOutputs[0], -motorOutputs[1]);
        updateOdometry();
    }

    public double[] calculateOutputs(double speed, double turn) {
        double[] wheelSetPoints = robotToWheels(speed, turn);
        VSetpoints[0] = wheelSetPoints[0];
        VSetpoints[1] = wheelSetPoints[1];
        double leftPositinRadians = left.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        double rightPositinRadians = right.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        double motorOutputLeft = motorControlLeftVelocity.PIDVelocity(leftPositinRadians, wheelSetPoints[0]); //TODO: check for mistakes
        double motorOutputRight = motorControlRightVelocity.PIDVelocity(rightPositinRadians, wheelSetPoints[1]);
        if (Math.abs(motorOutputLeft) < 0.1)
            motorOutputLeft = 0;
        if (Math.abs(motorOutputRight) < 0.1)
            motorOutputRight = 0;
        return new double[]{motorOutputLeft, motorOutputRight};
    }

    public double[] getRobotVelocities() {
        return wheelsToRobot(motorControlLeftVelocity.calculateDerivative(), motorControlRightVelocity.calculateDerivative());
    }

    private double[] robotToWheels(double linear, double angular) {
        double Vleft = (linear / WHEEL_RADIUS) - (angular * WHEEL_DISTANCE) / (2 * WHEEL_RADIUS);
        double Vright = (linear / WHEEL_RADIUS) + (angular * WHEEL_DISTANCE) / (2 * WHEEL_RADIUS);
        return new double[]{Vleft, Vright};
    }

    public double[] wheelsToRobot(double Vleft, double Vright) {
        double linear = (Vright + Vleft) * WHEEL_RADIUS / 2.0;
        double angular = (Vright - Vleft) * WHEEL_RADIUS / WHEEL_DISTANCE;
        return new double[]{linear, angular};
    }

    public void updateOdometry() {
        theta = 0;
        if (gyro == null) {
            set("gyro", "Not Working");
        } else {
            if (checkGyro) {
                checkGyro = false;
                gyro.reset();
            } else {
                set("gyro", "Working");
                theta = gyro.getAngle();
            }
        }
        encoders[0] = left.getEncoder().get();
        encoders[1] = right.getEncoder().get();
        log("encoder right: " + right.getEncoder().getRaw());
        log("encoder left: " + left.getEncoder().getRaw());
        leftMeters = ((encoders[0] - encodersPrev[0]) /512 ) * 2 * Math.PI * WHEEL_RADIUS;
        rightMeters = ((encoders[1] - encodersPrev[1]) / 512) * 2 * Math.PI * WHEEL_RADIUS;
        VOmegaReal = wheelsToRobot(motorControlLeftVelocity.calculateDerivative(), motorControlRightVelocity.calculateDerivative());
        distanceFromEncoders = (leftMeters + rightMeters) / 2.0;
        x += distanceFromEncoders * Math.sin(toRadians(theta)); //cos
        y += distanceFromEncoders * Math.cos(toRadians(theta)); //sin
        thetaRobotPrev = theta;
        encodersPrev[0] = encoders[0];
        encodersPrev[1] = encoders[1];
        odometry.setX(x);
        odometry.setY(y);
        odometry.setTheta(theta);
        odometry.setRightSetpoint(RSetpoint);
        odometry.setLeftSetpoint(LSetpoint);
        odometry.setDistance(distanceFromEncoders);
    }

    public void initGyro(Gyroscope gyro) {
        this.gyro = gyro;
        while (gyro.isCalibrating()) //log("Calibrating Gyro");
        offsetGyro = gyro.getYaw();

    }

    public void direct(double leftSpeed, double rightSpeed) {
        left.applyPower(leftSpeed);
        right.applyPower(rightSpeed);
    }

    public double toRadians(double degrees) {
        return (degrees / 180) * Math.PI;
    }

    public double toDegrees(double radians) {
        return radians * (180 / Math.PI);
    }
}
