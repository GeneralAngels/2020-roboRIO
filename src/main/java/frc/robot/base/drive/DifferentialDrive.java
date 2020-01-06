package frc.robot.base.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.base.Module;
import frc.robot.base.control.PID;
import frc.robot.base.utils.MotorGroup;

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
    public double WHEEL_DISTANCE = 0.51;
    public double WHEEL_RADIUS = 0.1016;
    public double MAX_V = 2;
    public double MAX_OMEGA = 3.14 * 2;
    public double gearRatio = 14.0 / 60.0;
    public double ENCODER_COUNT_PER_REVOLUTION = 512;
    public double ENCODER_TO_RADIAN = (Math.PI * 2) / (4 * ENCODER_COUNT_PER_REVOLUTION);
    public double setPointVPrev = 0;
    public double setPointOmegaPrev = 0;
    public double thetaRobotPrev = 0;
    public double x = 0;
    public double y = 0;
    public double MAX_WHEEL_VELOCITY = 20;
    public double leftMeters;
    public double rightMeters;
    public double[] VOmegaReal = {0, 0};
    public double[] VOmegaSetpoints = {0, 0};
    public double[] VSetpoints = {0, 0};
    ;
    public double[] encoders = {0, 0};
    public double[] encodersPrev = {0, 0};
    public double offsetGyro = 0;
    public double distanceFromEncoders = 0;
    public double theta = 0;
    public double[] motorOutputs = {0, 0};
    public double[] motorOutputsPrev = {0, 0};
    public MotorGroup<T> left = new MotorGroup<>("left"), right = new MotorGroup<>("right");
    protected Odometry odometry = new Odometry();
    public double battery = 12;
    double batteryPrev = 0;
    boolean checkGyro = true;


    public DifferentialDrive() {

        super("drive");
        motorControlLeftVelocity = new PID(0, 0, 0, 0.55);
        motorControlRightVelocity = new PID(0, 0, 0, 0.55);
        motorControlLeftPosition = new PID(3, 0.1, 0.2, 0);
        motorControlRightPosition = new PID(3, 0.1, 0.2, 0);

        addSlave(odometry);
        addSlave(motorControlLeftPosition);
    }

    public void setNoPID(double speed, double turn) {
        direct(noPIDCalculateLeft(speed, turn), noPIDCalculateRight(speed, turn));
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

    public void setTank(double Vl, double Vr) {
        if (Math.abs(Vl) < 0.1)
            Vl = 0;
        if (Math.abs(Vr) < 0.1)
            Vr = 0;
        VSetpoints[0] = Vl * MAX_WHEEL_VELOCITY;
        VSetpoints[1] = Vr * MAX_WHEEL_VELOCITY;
        VOmegaSetpoints = wheelsToRobot(VSetpoints[0], VSetpoints[1]);
        motorOutputs = calculateOutputs(VOmegaSetpoints[0], VOmegaSetpoints[1]);
        battery = DriverStation.getInstance().getBatteryVoltage();
        battery = (0.5 * battery) + (0.5 * batteryPrev);
        if (battery > 12.0)
            battery = 12.0;
        batteryPrev = battery;
        motorOutputsPrev[0] = motorOutputs[0];
        motorOutputsPrev[1] = motorOutputs[1];
        direct(-motorOutputs[0] / (battery), -motorOutputs[1] / (battery));
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
        setTank((-2 - motorControlLeftPosition.pidPosition(angle, initialAngle + target)), (2 + motorControlRightPosition.pidPosition(angle, initialAngle + target)));
    }

    public void driveStraightPID() {
        int initialSpeed = 4;
        setTank(initialSpeed + motorControlLeftPosition.pidPosition(toRadians(gyro.getYaw()), 0), initialSpeed - motorControlRightPosition.pidPosition(toRadians(gyro.getYaw()), 0));
    }

    public void set(double setpointV, double setpointOmega, boolean auto) {
        if (auto) {
            motorControlLeftVelocity.setPIDF(0.5, 0.3, 0, 0.5);
            motorControlRightVelocity.setPIDF(0.5, 0.3, 0, 0.5);
            log("autonomous mode");
        } else {
            motorControlLeftVelocity.setPIDF(0, 0, 0, 0.55);
            motorControlRightVelocity.setPIDF(0, 0, 0, 0.55);
            log("TeleOp");
        }
        if (Math.abs(setpointV) < 0.2) setpointV = 0;
        if (Math.abs(setpointOmega) < 0.2) setpointOmega = 0;
        double[] motorOutputs = calculateOutputs(setpointV, setpointOmega);
        battery = DriverStation.getInstance().getBatteryVoltage();
        battery = (0.5 * battery) + (0.5 * batteryPrev);
        if (battery > 12.0)
            battery = 12.0;
        batteryPrev = battery;
        motorOutputsPrev[0] = motorOutputs[0];
        motorOutputsPrev[1] = motorOutputsPrev[1];
        setPointVPrev = setpointV;
        setPointOmegaPrev = setpointOmega;
        direct(-(motorOutputs[0] / battery), -(motorOutputs[1] / battery));
        updateOdometry();
    }


    public double[] calculateOutputs(double speed, double turn) {
        double[] wheelSetPoints = robotToWheels(speed, turn);
        VSetpoints[0] = wheelSetPoints[0];
        VSetpoints[1] = wheelSetPoints[1];
        double leftPositinRadians = left.getEncoder().get() * ENCODER_TO_RADIAN * gearRatio;
        double rightPositinRadians = right.getEncoder().get() * ENCODER_TO_RADIAN * gearRatio;
        double motorOutputLeft = motorControlLeftVelocity.pidVelocity(leftPositinRadians, wheelSetPoints[0]); //TODO: check for mistakes
        double motorOutputRight = motorControlRightVelocity.pidVelocity(rightPositinRadians, wheelSetPoints[1]);
        if (Math.abs(motorOutputLeft) < 0.1)
            motorOutputLeft = 0;
        if (Math.abs(motorOutputRight) < 0.1)
            motorOutputRight = 0;
        return new double[]{motorOutputLeft, motorOutputRight};
    }

    public double[] getRobotVelocities() {
        return wheelsToRobot(motorControlLeftVelocity.getDerivative(), motorControlRightVelocity.getDerivative());
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
        leftMeters = (encoders[0] - encodersPrev[0]) * ENCODER_TO_RADIAN * WHEEL_RADIUS * 2 * gearRatio;
        rightMeters = (encoders[1] - encodersPrev[1]) * ENCODER_TO_RADIAN * WHEEL_RADIUS * 2 * gearRatio;
        VOmegaReal = wheelsToRobot(motorControlLeftVelocity.derivative, motorControlRightVelocity.derivative);
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
        while (gyro.isCalibrating()) log("Calibrating Gyro");
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
