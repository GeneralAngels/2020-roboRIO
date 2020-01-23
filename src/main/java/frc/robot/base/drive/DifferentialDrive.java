package frc.robot.base.drive;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
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

import static java.lang.Thread.getAllStackTraces;
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
    public PathFollower follower;
    public Trajectory trajectory;
    public double battery = 12;
    public double batteryPrev = 0;
    public boolean checkGyro = true;
    public boolean isAuto = false;
    private PowerDistributionPanel pdp;

    private double[] motorOutputs = new double[2];
    private double[] motorOutputsPrev = new double[2];

    private Trajectory.State basePose = null;

    private long trajectoryStart = 0;

    public DifferentialDrive() {
        super("drive");
        pdp = new PowerDistributionPanel(0);
        motorControlLeftVelocity = new PID("pid_left_velocity", 0.33, 0, 1, 0);
        motorControlRightVelocity = new PID("pid_right_velocity", 0.33, 0, 1, 0);
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
        gyro.resetGyro();
    }
    public void printEncoders(){
        log("leftEncoder: "+left.getEncoder().getRaw()+"rightEncoder: "+right.getEncoder().getRaw());
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
            Trajectory.State goal = this.trajectory.sample(time);
            RamseteController controller = new RamseteController();
            Pose2d currentPose = getPose(); //x, y, rotation

            DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7112); //parameter is the wheel to wheel distance
            ChassisSpeeds adjustedSpeeds = controller.calculate(currentPose, goal);
            DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
            double leftVelocity = wheelSpeeds.leftMetersPerSecond;
            double rightVelocity = wheelSpeeds.rightMetersPerSecond;
            double[] errorVector = {goal.poseMeters.getTranslation().getX() - x, goal.poseMeters.getTranslation().getX() - y};
            double errorTheta = goal.poseMeters.getRotation().getDegrees() - theta;
            double correctionAngle = Math.atan2(errorVector[1], errorVector[0]);
            double errorDistance = Math.sqrt(Math.pow(goal.poseMeters.getTranslation().getY() - y, 2) + Math.pow(goal.poseMeters.getTranslation().getX() - x, 2))*errorVector[0]/Math.abs(errorVector[0]);
            log("errorDistance: "+errorDistance);
            if (true){//!isDone(getPose(), trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters) || goal.velocityMetersPerSecond + (errorDistance * 0.7) > 0.05) {
                set(goal.velocityMetersPerSecond + (errorDistance * 0.9), (goal.curvatureRadPerMeter * goal.velocityMetersPerSecond) + (errorTheta * 0.1));
                log("done");
            }
            else {
                setMode(false);
            }
        } else {
            direct(motorOutputs[0], motorOutputs[1]);
        }
        updateOdometry();
    }


    public Pose2d getPose(){
        return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    }

    public boolean isDone(Pose2d current, Pose2d end){
        double tolerance = 0.05;
        double errorX = Math.abs(current.getTranslation().getX() - end.getTranslation().getX());
        double errorY = Math.abs(current.getTranslation().getY() - end.getTranslation().getY());
        double errorTheta = Math.abs(current.getRotation().getDegrees() - Math.abs(current.getRotation().getDegrees()));
        return errorX < tolerance & errorY < tolerance & errorTheta < 5;
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

    public double[] xyTofs(double errorX, double errorY){
        double errorFront, errorSide;
        if(Math.cos(toRadians(theta))==0) {
            errorFront = errorY;
            errorSide = errorX;
        }
        else if(Math.sin(toRadians(theta))==0){
            errorFront = errorX;
            errorSide = errorY;
        }
        else{
            errorFront = (errorX/Math.cos(toRadians(theta))) + (errorY/Math.sin(toRadians(theta)));
            errorSide = (errorX/Math.sin(toRadians(theta))) + (errorY/Math.cos(toRadians(theta)));
        }
        return new double[]{errorFront, errorSide};
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
        log("battery: "+pdp.getVoltage());
        //motorOutputsPrev[0] = motorOutputs[0];
        //motorOutputsPrev[1] = motorOutputs[1];
        motorOutputs = calculateOutputs(setpointV, setpointOmega);
        //setpointVPrevious = setpointV;
        //setpointOmegaPrevious = setpointOmega;
        direct(motorOutputs[0]/12.0, -motorOutputs[1]/12.0);
        updateOdometry();
    }

    public double[] calculateOutputs(double speed, double turn) {
        double[] wheelSetPoints = robotToWheels(speed, turn);
        VSetpoints[0] = wheelSetPoints[0];
        VSetpoints[1] = wheelSetPoints[1];
        double leftPositionMeters = left.getEncoder().getRaw() * ENCODER_TO_RADIAN*WHEEL_RADIUS; //radians to meters added
        double rightPositionMeters = right.getEncoder().getRaw() * ENCODER_TO_RADIAN*WHEEL_RADIUS;
        double motorOutputLeft = motorControlLeftVelocity.PIDVelocity(leftPositionMeters, wheelSetPoints[0]);
        double motorOutputRight = motorControlRightVelocity.PIDVelocity(rightPositionMeters, wheelSetPoints[1]);
//        if (Math.abs(motorOutputLeft) < 0.1)
//            motorOutputLeft = 0;
//        if (Math.abs(motorOutputRight) < 0.1)
//            motorOutputRight = 0;
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
        //log("encoder right: " + right.getEncoder().getRaw());
        //log("encoder left: " + left.getEncoder().getRaw());
        leftMeters = ((encoders[0] - encodersPrev[0]) / 2048) * 2 * Math.PI * WHEEL_RADIUS; // 2048 instead of 512
        rightMeters = ((encoders[1] - encodersPrev[1]) / 2048) * 2 * Math.PI * WHEEL_RADIUS;// 2048 instead of 512
        distanceFromEncoders = (leftMeters + rightMeters) / 2.0;
        x += distanceFromEncoders * Math.cos(toRadians(theta)); //sin
        y += distanceFromEncoders * Math.sin(toRadians(theta)); //cos
        log("x: "+x+" y: "+y+" theta: "+theta);
        //log("theta: " + theta);
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

    public double toRadians(double degrees) {
        return (degrees / 180) * Math.PI;
    }

    public double toDegrees(double radians) {
        return radians * (180 / Math.PI);
    }
}
