package frc.robot.bobot.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.control.PID;
import org.json.JSONObject;

public class DifferentialDrive<T extends SpeedController> extends Subsystem {
    public static final String LEFT = "left";
    public static final String RIGHT = "right";
    public static final String VELOCITY = "v";
    public static final String OMEGA = "w";
    public static final String GYRO = "gyro";
    public static final String ODOMETRY = "odometry";
    public double Lsetpoint = 0;
    public double Rsetpoint = 0;
    public PID motorControlLeft;
    public PID motorControlRight;
    public PID motorControlLeftP;
    public PID motorControlRightP;
    public AHRS gyro;
    public double WHEEL_DISTANCE = 0.51;
    public double WHEEL_RADIUS = 0.1016;
    public double MAX_V = 1;

    public double MAX_OMEGA = 3.14 * 2;
    public double[] realVOmega;
    public double gearRatio = 14.0 / 60.0;
    public double ENCODER_COUNT_PER_REVOLUTION = 512;
    public double ENCODER_TO_RADIAN = (Math.PI * 2) / (4 * ENCODER_COUNT_PER_REVOLUTION);
    public double setPointVPrev = 0;
    public double setPointOmegaPrev = 0;
    public double thetaRobotPrev = 0;
    public double x = 0;
    public double y = 0;
    public double rightEncPrev = 0;
    public double leftEncPrev = 0;
    public double MAX_WHEEL_VELOCITY = 20;
    public double lastSetPointsR = 0;
    public double lastSetPointsL = 0;
    public double lastav = 0;
    public double leftMeters;
    public double rightMeters;
    public double outputLeft = 0;
    public double outputRight = 0;
    public double[] VOmegaReal = {0, 0};
    public double[] VOmega = {0, 0};
    public double Vleft = 0;
    public double Vright = 0;
    public double leftMetersPrev = 0;
    public double rightMetersPrev = 0;
    public double[] encoderMeters = {0, 0};
    public double[] encoders = {0, 0};
    public double[] encodersPrev = {0, 0};
    public double offsetGyro = 0;
    public double distance = 0;
    public double theta = 0;
    public double motorOutputLeft = 0;
    public double motorOutputRight = 0;
    public boolean check = true;
    public double currentMetersRight = 0;
    public double currentMetersLeft = 0;
    public PID pid;
    protected Drivebox<T> left = new Drivebox<>(), right = new Drivebox<>();
    protected Odometry odometry = new Odometry();
    double motorOutputLeftPrev = 0;
    double motorOutputRightPrev = 0;

    public DifferentialDrive() {
        motorControlLeft = new PID();
        motorControlRight = new PID();
        motorControlLeftP = new PID();
        motorControlRightP = new PID();
        pid = new PID();
        motorControlLeft.setPIDF(0, 0.2, 0.2, 0.5);
        motorControlRight.setPIDF(0, 0.2, 0.2, 0.5);
        motorControlLeftP.setPIDF(2.5, 0.4, 0.2, 0);
        motorControlRightP.setPIDF(2, 0.4, 0.2, 0);
        pid.setPIDF(1, 0.1, 0, 0);
    }

    public static double noPIDCalculateRight(double speed, double turn) {
        if (speed == 0) {
            return turn;
        } else {
            return (speed + turn);
        }
    }

    public static double noPIDCalculateLeft(double speed, double turn) {
        if (speed == 0) {
            return turn;
        } else {
            return (turn - speed);
        }
    }

    public void hatchAlign(double errorAngle) {
        if (Math.abs(errorAngle) >= 0.6) {
            pid.ki = 0.1;
            pid.kp = 0.8;
        } else {
            pid.kp = 1.7;
            pid.ki = 0.45;
        }
        double p = pid.pidPosition(0, errorAngle);
        log("hatch_align_power", "" + p);
        direct(p, -p);
    }

    public void setAutonomous(double v, double w) {
        set(v, w);
    }

    public void setStickNoPID(double speed, double turn) {
        double l, r;
        l = noPIDCalculateLeft(speed, turn);
        r = noPIDCalculateRight(speed, turn);
        direct(l, r);
    }

    public void setTank(double Vl, double Vr) {
        if (Math.abs(Vl) < 0.1)
            Vl = 0;
        if (Math.abs(Vr) < 0.1)
            Vr = 0;
        Vl = Vl * MAX_WHEEL_VELOCITY;
        Vr = Vr * MAX_WHEEL_VELOCITY;
        Vleft = Vl;
        Vright = Vr;
        VOmega = wheelsToRobot(Vleft, Vright);
        encoders[0] = left.getEncoder().getRaw();
        encoders[1] = right.getEncoder().getRaw();
        double encoderLeft = encoders[0] * ENCODER_TO_RADIAN * gearRatio;
        double encoderRight = encoders[1] * ENCODER_TO_RADIAN * gearRatio;
        double motorOutputLeft = motorControlLeft.pidVelocity(encoderLeft, Vl);
        double motorOutputRight = motorControlRight.pidVelocity(encoderRight, Vr);
        if (Math.abs(motorOutputLeft) < 0.1)
            motorOutputLeft = 0;
        if (Math.abs(motorOutputRight) < 0.1)
            motorOutputRight = 0;
        motorOutputLeftPrev = motorOutputLeft;
        motorOutputRightPrev = motorOutputRight;
        direct(motorOutputLeft / 12, motorOutputRight / 12);
        updateOdometry();
    }

    public void set(double speed, double turn) {
        VOmega[0] = speed;
        VOmega[1] = turn;
        double[] V = robotToWheels(VOmega[0], VOmega[1]);
        double setpointV = VOmega[0];
        double setpointOmega = VOmega[1];
        if (Math.abs(setpointV) < 0.2) setpointV = 0;
        if (Math.abs(setpointOmega) < 0.2) setpointOmega = 0;
        encoders[0] = left.getEncoder().getRaw();
        encoders[1] = right.getEncoder().getRaw();
        double encoderLeft = encoders[0] * ENCODER_TO_RADIAN * gearRatio;
        double encoderRight = encoders[1] * ENCODER_TO_RADIAN * gearRatio;
        double motorOutputLeft = motorControlLeft.pidVelocity(encoderLeft, V[0]);
        double motorOutputRight = motorControlRight.pidVelocity(encoderRight, V[1]);
        if (Math.abs(motorOutputLeft) < 0.1)
            motorOutputLeft = 0;
        if (Math.abs(motorOutputRight) < 0.1)
            motorOutputRight = 0;
        motorOutputLeftPrev = motorOutputLeft;
        motorOutputRightPrev = motorOutputRight;
        setPointVPrev = setpointV;
        setPointOmegaPrev = setpointOmega;
        direct(motorOutputLeft / 12.0, motorOutputRight / 12.0);
        updateOdometry();
    }

    public double[] accControl(double VL, double VR) {
        double maxAcc = 0.3;
        double av = (VL + VR) / 2;
        double error = av - this.lastav;
        if (error > maxAcc) {
            this.lastSetPointsR = this.lastSetPointsR + 0.15;
            this.lastSetPointsL = this.lastSetPointsL + 0.15;
            return new double[]{this.lastSetPointsL, this.lastSetPointsR};
        } else if (error < -maxAcc) {
            this.lastSetPointsR = this.lastSetPointsR - 0.15;
            this.lastSetPointsL = this.lastSetPointsL - 0.15;
            return new double[]{this.lastSetPointsL, this.lastSetPointsR};
        }
        this.lastSetPointsL = VL;
        this.lastSetPointsR = VR;
        this.lastav = av;
        return new double[]{VL, VR};
    }

    public double[] calculateOutputs(double speed, double turn) {
        double[] wheelSetPoints = robotToWheels(speed, turn);
        Rsetpoint = wheelSetPoints[1];
        Lsetpoint = wheelSetPoints[0];
        double encoderLeft = left.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        double encoderRight = right.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        double motorOutputLeft = motorControlLeft.pidVelocity(encoderLeft, wheelSetPoints[0]);
        double motorOutputRight = motorControlRight.pidVelocity(encoderRight, wheelSetPoints[1]);
        return new double[]{motorOutputLeft, motorOutputRight};
    }

    public double[] getRobotVelocities() {
        return wheelsToRobot(motorControlLeft.getDerivative(), motorControlRight.getDerivative());
    }

    private double[] robotToWheels(double linear, double angular) {
        double Vleft = (linear / WHEEL_RADIUS) - (angular * WHEEL_DISTANCE) / (2 * WHEEL_RADIUS);
        double Vright = (linear / WHEEL_RADIUS) + (angular * WHEEL_DISTANCE) / (2 * WHEEL_RADIUS);
        return new double[]{Vleft, Vright};
    }

    public double[] wheelsToRobot(double Vleft, double Vright) {
        double linear = (Vright + Vleft) * WHEEL_RADIUS / 2.0;
        double angular = (Vleft - Vright) * WHEEL_RADIUS / WHEEL_DISTANCE;
        return new double[]{linear, angular};
    }

    public void updateOdometry() {
        theta = 0;
        if (gyro == null) {
            log("gyro", "Not Working");
        } else {
            log("gyro", "Working");
            theta = gyro.getYaw() - offsetGyro;
        }

        leftMeters = (encoders[0] - encodersPrev[0]) * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
        rightMeters = (encoders[1] - encodersPrev[1]) * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
        VOmegaReal = wheelsToRobot(motorControlLeft.derivative, motorControlRight.derivative);
        distance = (leftMeters + rightMeters) / 2.0;
        x += distance * Math.cos(toRadians(theta));
        y += distance * Math.sin(toRadians(theta));
        thetaRobotPrev = theta;
        encodersPrev[0] = encoders[0];
        encodersPrev[1] = encoders[1];
        odometry.setX(x);
        odometry.setY(-y);
        odometry.setTheta(theta);
        odometry.setRightSetpoint(Rsetpoint);
        odometry.setLeftSetpoint(Lsetpoint);
    }

    public void preClimb() {
        if (check) {
            currentMetersLeft = left.getEncoder().getRaw() * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
            check = false;
            currentMetersRight = right.getEncoder().getRaw() * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
        }
        motorOutputLeft = motorControlLeftP.pidPosition(leftMeters, currentMetersLeft + 0.13);
        motorOutputRight = motorControlRightP.pidPosition(rightMeters, currentMetersRight + 0.13);
        if (Math.abs(motorOutputLeft) < 0.2)
            motorOutputLeft = 0;
        if (Math.abs(motorOutputRight) < 0.2)
            motorOutputRight = 0;
        direct(motorOutputLeft, motorOutputRight);
        leftMeters = left.getEncoder().getRaw() * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
        rightMeters = right.getEncoder().getRaw() * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
    }

    public void initGyro(AHRS gyro) {
        this.gyro = gyro;
        while (gyro.isCalibrating()) log("Calibrating Gyro");
        offsetGyro = gyro.getYaw();

    }

    public void direct(double leftSpeed, double rightSpeed) {
        left.set(leftSpeed);
        right.set(rightSpeed);
    }

    public double toRadians(double degrees) {
        return (degrees / 180) * Math.PI;
    }

    @Override
    public JSONObject toJSON() {
        JSONObject returnObject = super.toJSON();
        try {
            //            returnObject.put("v_robot_setpoint", VOmega[0]);
            //            returnObject.put("omega_robot_setpoint", VOmega[1]);
            //            returnObject.put("v_robot_real", VOmegaReal[0]);
            //            returnObject.put("omega_robot_real", VOmegaReal[1]);
            returnObject.put("left_encoder", encoders[0]);
            returnObject.put("right_encoder", encoders[1]);
            returnObject.put("v_left_real", motorControlLeft.derivative);
            returnObject.put("v_right_real", motorControlRight.derivative);
            returnObject.put("v_left_setpoint", Vleft);
            returnObject.put("v_right_setpoint", Vright);
            returnObject.put("output_left", outputLeft);
            returnObject.put("output_right", outputRight);
            returnObject.put(ODOMETRY, odometry.toJSON());
        } catch (Exception ignored) {
        }
        return returnObject;
    }
}
