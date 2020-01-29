package frc.robot.base.drive;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.base.Module;
import frc.robot.base.control.PID;
import frc.robot.base.control.path.PathManager;
import frc.robot.base.utils.MotorGroup;
import org.opencv.core.Mat;

import java.util.List;

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
    public double WHEEL_DISTANCE = 0.7112;
    public double WHEEL_RADIUS = 0.0762;
    public double MAX_V = 1.0;
    public double MAX_OMEGA = 3.14 * 2;
    public double gearRatio = 1.0;
    public double ENCODER_COUNT_PER_REVOLUTION = 512;
    public double ENCODER_TO_RADIAN = (Math.PI * 2) / (4 * ENCODER_COUNT_PER_REVOLUTION);
    public double setpointVPrevious = 0;
    public double setpointOmegaPrevious = 0;
    public double thetaRobotPrev = 0;
    public double x = 0;
    public double y = 0;
    public double MAX_WHEEL_VELOCITY = 20; //    4/WHEEL_RADIUS
    public double leftMeters;
    public double rightMeters;
    public double leftMetersPrev;
    public double rightMetersPrev;
    public double[] VOmegaReal = {0, 0};
    public double[] VOmegaSetpoints = {0, 0};
    public double[] VSetpoints = {0, 0};
    public double[] encoders = {0, 0};
    public double[] encodersPrev = {0, 0};
    public double offsetGyro = 0;
    public double distanceFromEncoders = 0;
    public double theta = 0;
    public double xPrev = 0;
    public MotorGroup<T> left = new MotorGroup<>("left"), right = new MotorGroup<>("right");
    public Odometry odometry = new Odometry();
    public PathManager follower;
    public Trajectory trajectory;
    public double battery = 12;
    public double batteryPrev = 0;
    public boolean checkGyro = true;
    public boolean isAuto = false;
    private PowerDistributionPanel pdp;
    private List<Trajectory.State> trajectoryStates;
    private double[] motorOutputs = new double[2];
    private double[] motorOutputsPrev = new double[2];
    public int trajectoryIndex;
    private Trajectory.State basePose = null;
    public boolean lastIndex = true;
    private long trajectoryStart = 0;
    public double Kv = 1;
    public double Komega = 2; // Note that low values (for example 0.33) means it should get to the setpoint in ~3 seconds!
    public double Kcurv = 0;


    public DifferentialDrive() {
        super("drive");
        pdp = new PowerDistributionPanel(0);
        motorControlLeftVelocity = new PID("pid_left_velocity", 0, 0.03, 0, 0.24);
        motorControlRightVelocity = new PID("pid_right_velocity", 0, 0.03, 0, 0.24);
        motorControlLeftPosition = new PID("pid_left_position", 3, 0.1, 0.2, 0);
        motorControlRightPosition = new PID("pid_right_position", 3, 0.1, 0.2, 0);
        follower = new PathManager(this);
        enslave(follower);
        enslave(left);
        enslave(right);
        enslave(odometry);
        enslave(motorControlLeftVelocity);
        enslave(motorControlRightVelocity);
        enslave(motorControlLeftPosition);
        enslave(motorControlRightPosition);
        this.trajectory = follower.createPath();
        this.trajectoryStates = trajectory.getStates();
        gyro = new Gyroscope();
        initGyro(gyro);
        setMode(true);
        gyro.resetGyro();
        trajectoryIndex = 1;
    }

    public void printEncoders() {
        log("leftEncoder: " + left.getEncoder().getRaw() + "rightEncoder: " + right.getEncoder().getRaw());
    }

    public void setTrajectory(Trajectory trajectory) {
        this.basePose = new Trajectory.State();
        this.trajectory = trajectory;
        this.trajectoryStart = millis();

    }

    public void setMode(boolean trajectory) {
        if (trajectory)
            this.trajectoryStart = millis();
        this.isAuto = trajectory;
    }

    public void loop(double time) {
        if (isAuto) {
            Trajectory.State goal = this.trajectory.sample(time);
            //log("curve: " + goal.curvatureRadPerMeter);
            double[] speeds = calcErrors(goal, goal); // Sent goal twice because there is no meaning to prevGoal
            if (true) {//!isDone(getPose(), trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters) || goal.velocityMetersPerSecond + (errorDistance * 0.7) > 0.05) {
                //set(speeds[0] * 8, speeds[1] * 0.7);
                set(goal.velocityMetersPerSecond + speeds[0], (goal.velocityMetersPerSecond * goal.curvatureRadPerMeter) + speeds[1]);
                log("done");
            } else {
                setMode(false);
            }
        } else {
            direct(motorOutputs[0], motorOutputs[1]);
        }
        updateOdometry();
    }

    public void pathFollowingProcedure() {
        Trajectory.State currentGoal = trajectoryStates.get(trajectoryIndex);
        Trajectory.State previousGoal = trajectoryStates.get(trajectoryIndex - 1);
        double curvature = currentGoal.curvatureRadPerMeter;
        double[] errors = calcErrors(currentGoal, previousGoal);
        super.set("errorTheta", "" + errors[2]);
        super.set("trajectoryIndex", "" + trajectoryIndex);

        if (trajectoryIndex < trajectoryStates.size() - 1) { //length of the trajectory
            double desiredVelocity = MAX_V - curvature * Kcurv;
            set(desiredVelocity, errors[2] * Komega);
            log("not last point");
            if (errors[0] < errors[1]) {
                trajectoryIndex++;
            }
        } else {
            //lastIndex = true;
            //double[] speeds = calcErrors(trajectoryStates.get(trajectoryIndex));
            if (!isDone(getPose(), trajectoryStates.get(trajectoryIndex).poseMeters)) {
                set(errors[0] * Kv, errors[2] * Komega);
                log("last point");
            } else {
                set(0, 0);
                log("done");
            }
        }
    }

    public double[] calcErrors(Trajectory.State currentGoal, Trajectory.State previousGoal) {
        double errorXCurrent, errorYCurrent, errorYPrev, errorXPrev;
        errorXCurrent = currentGoal.poseMeters.getTranslation().getX() - x;
        errorYCurrent = currentGoal.poseMeters.getTranslation().getY() - y;
        errorXPrev = previousGoal.poseMeters.getTranslation().getX() - x;
        errorYPrev = previousGoal.poseMeters.getTranslation().getY() - y;
        double errorTheta = Math.atan2(errorYCurrent, errorXCurrent) - toRadians(theta);
        if (errorTheta > 3.141)
            errorTheta -= 6.282;
        if (errorTheta < -3.141)
            errorTheta += 6.282;

        double errorPositionCurrent = Math.sqrt(Math.pow(errorXCurrent, 2) + Math.pow(errorYCurrent, 2));
        double errorPositionPrev = Math.sqrt(Math.pow(errorXPrev, 2) + Math.pow(errorYPrev, 2));
        double[] errors = new double[]{errorPositionCurrent, errorPositionPrev, errorTheta};
        log("\n\nerrorTheta:" + errorTheta);

        //double dir = sign(Math.cos(errors[1]));
//        double dir = (trajectoryIndex < trajectoryStates.size() - 1) ? 1 : -1;
//        double v = errors[0] * -dir;
        //double omega = errorTheta;
        //return new double[] {v, omega};
        return errors;
    }

//    public double[] calcErrors(Trajectory.State goal){
//        double errorX, errorY;
//        errorX = goal.poseMeters.getTranslation().getX() - x;
//        errorY = goal.poseMeters.getTranslation().getY() - y;
//        //double errorAng = Math.atan2(errorY, errorX);
//        double errorAng = Math.atan2(goal.poseMeters.getTranslation().getY(), goal.poseMeters.getTranslation().getX());
//        double errorTheta = toRadians(goal.poseMeters.getRotation().getDegrees() - theta);
//        if (errorAng > 3.141)
//            errorAng -= 6.282;
//        if (errorAng < -3.141)
//            errorAng += 6.282;
//
//        double errorPosition = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2));
//        double[] errors =  new double[] {errorPosition, errorAng};
//        log("\n\nerrorTheta:"+errorTheta);
//        log("errorAngle: "+errorAng+"\n\n");
//        //double dir = sign(Math.cos(errors[1]));
//
////        double dir = (trajectoryIndex < trajectoryStates.size() - 1) ? 1 : -1;
////        double v = errors[0] * -dir;
//
//
//        //double v = errors[0];
//        //double omega = errorTheta;
//        //return new double[] {v, omega};
//        return new double[] {errorPosition, errorTheta};
//    }


    public void followSplineNoPID(double[] splinex, double[] spliney, double t) {
        double[] dsplinex = follower.derivePolynom(splinex);
        double[] dspliney = follower.derivePolynom(spliney);
        double dx = follower.put(dsplinex, t);
        double dy = follower.put(dspliney, t);
        log("*dx, dy " + dx + ", " + dy);
        direct(dx, dy);
    }


    public double sign(double num) {
        return (num < 0) ? -1 : 1;
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    }

    public boolean isDone(Pose2d current, Pose2d end) {
        double tolerance = 0.2;
        double errorX = Math.abs(current.getTranslation().getX() - end.getTranslation().getX());
        double errorY = Math.abs(current.getTranslation().getY() - end.getTranslation().getY());
        double errorTheta = Math.abs(current.getRotation().getDegrees() - end.getRotation().getDegrees());
        return errorX < tolerance && errorY < tolerance && errorTheta < 5;
    }

    public void setNoPID(double speed, double turn) {
        motorOutputs[0] = (turn + speed);
        motorOutputs[1] = (turn - speed);
        direct(motorOutputs[0], motorOutputs[1]);
    }

    public static double noPIDCalculateRight(double speed, double turn) {
        return (speed - turn);
    }

    public static double noPIDCalculateLeft(double speed, double turn) {
        return (speed + turn);
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

    @Deprecated
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

//    public void driveStraightPID() {
//        int initialSpeed = 4;
//        setTank(initialSpeed + motorControlLeftPosition.PIDPosition(toRadians(gyro.getYaw()), 0), initialSpeed - motorControlRightPosition.PIDPosition(toRadians(gyro.getYaw()), 0));
//    }

    public void set(double setpointV, double setpointOmega) {
        if (Math.abs(setpointV) < 0.05)
            setpointV = 0;
        if (Math.abs(setpointOmega) < 0.05)
            setpointOmega = 0;
        motorOutputs = calculateOutputs(setpointV, setpointOmega);
        //direct(motorOutputs[0]/12.0, -motorOutputs[1]/12.0);
        direct(motorOutputs[0] / pdp.getVoltage(), -motorOutputs[1] / pdp.getVoltage());
        updateOdometry();
    }


    public double[] calculateOutputs(double speed, double turn) {
        double[] wheelSetPoints = robotToWheels(speed, turn);
        VSetpoints[0] = wheelSetPoints[0];
        VSetpoints[1] = wheelSetPoints[1];
        double leftPositionRadians = left.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        double rightPositionRadians = right.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        super.set("setpointLeft", "" + wheelSetPoints[0]);
        double motorOutputLeft = motorControlLeftVelocity.PIDVelocity(leftPositionRadians, wheelSetPoints[0]);
        double motorOutputRight = motorControlRightVelocity.PIDVelocity(rightPositionRadians, wheelSetPoints[1]);
        return new double[]{motorOutputLeft, motorOutputRight};
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
        theta = gyro.getAngle();
        encoders[0] = left.getEncoder().getRaw();
        encoders[1] = right.getEncoder().getRaw();
        leftMeters = ((encoders[0] - encodersPrev[0]) / 2048) * 2 * Math.PI * WHEEL_RADIUS; // 2048 instead of 512
        rightMeters = ((encoders[1] - encodersPrev[1]) / 2048) * 2 * Math.PI * WHEEL_RADIUS;// 2048 instead of 512
        distanceFromEncoders = (leftMeters + rightMeters) / 2.0;
        x += distanceFromEncoders * Math.cos(toRadians(theta)); //sin
        y += distanceFromEncoders * Math.sin(toRadians(theta)); //cos
        log("x: " + x + " y: " + y + " theta: " + theta);
        xPrev = x;
        leftMetersPrev = leftMeters;
        rightMetersPrev = rightMeters;
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
        offsetGyro = gyro.getAngle();
    }

    public void direct(double leftSpeed, double rightSpeed) {
        left.applyPower(leftSpeed);
        right.applyPower(rightSpeed);
    }

    public void directLeft(double leftSpeed) {
        left.applyPower(leftSpeed);
    }

    public double toRadians(double degrees) {
        return (degrees / 180) * Math.PI;
    }

    public double toDegrees(double radians) {
        return radians * (180 / Math.PI);
    }

    public void splineFollower(double[] coefsLeft, double coefsRight) {

    }
}
