package frc.robot.bobot.drive;

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
    public Gyroscope gyro;
    public double WHEEL_DISTANCE = 0.58;
    public double WHEEL_RADIUS = 0.105;
    public double MAX_V = 1.2;
//    for good results max omega = max_v * 2 /wheel_distance
    public double MAX_OMEGA = MAX_V * 2 / WHEEL_DISTANCE ;
    public double[] realVOmega;
    public double ENCODER_COUNT_PER_REVOLUTION = 500;
    public double ENCODER_TO_RADIAN = (Math.PI * 2) / (4 * ENCODER_COUNT_PER_REVOLUTION);
    public double setPointVPrev = 0;
    public double setPointOmegaPrev = 0;
    public double thetaRobotPrev = 0;
    public double x = 0;
    public double y = 0;
    public double rightEncPrev = 0;
    public double leftEncPrev = 0;
    public double MAX_WHEEL_VELOCITY = 10;
    protected Drivebox<T> left = new Drivebox<>(), right = new Drivebox<>();
    protected Odometry odometry = new Odometry();

    public DifferentialDrive() {
        motorControlLeft = new PID();
        motorControlRight = new PID();
        motorControlLeft.setPIDF(0, 8/0.08, 0, 0.4);
        motorControlRight.setPIDF(0, 0.08, 0, 0.4);
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

    public void setAutonomous(double v, double w) {
        // If limiting is needed
        // set(v/MAX_V,w/MAX_OMEGA);
        // If not
        set(v, w);
    }

    public void setStickNoPID(double speed, double turn) {
        double l,r;
        l=noPIDCalculateLeft(speed, turn);
        r=noPIDCalculateRight(speed, turn);
//        log("L "+l+" R "+r);
        direct(l,r);
    }

    public void set(double speed, double turn) {
        double setpointV = speed;
        double setpointOmega = turn;
        setpointV = (setpointV * 0.5) + (setPointVPrev * 0.5);
        setpointOmega = (setpointOmega * 0.5) + (setPointOmegaPrev * 0.5);
        if (Math.abs(setpointV) < 0.1) setpointV = 0;
        if (Math.abs(setpointOmega) < 0.1) setpointOmega = 0;
        double[] motorOutput = calculateOutputs(setpointV * MAX_V, setpointOmega * MAX_OMEGA);
        realVOmega = getRobotVelocities();
//        log("Il:" + motorControlLeft.getIntegral() + " Ir:" + motorControlRight.getIntegral() + " Vr=" + realVOmega[0] + " Left=" + motorControlLeft.getDerivative() + " Right=" + motorControlRight.getDerivative());

        setPointVPrev = setpointV;
        setPointOmegaPrev = setpointOmega;
        direct(motorOutput[0] / 12, motorOutput[1] / 12);
        updateOdometry();
    }

    public double[] calculateOutputs(double speed, double turn) {
        double[] wheelSetPoints = robotToWheels(speed, turn);
        Rsetpoint = wheelSetPoints[1];
        Lsetpoint = wheelSetPoints[0];
        double encoderLeft = left.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        double encoderRight = right.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        double motorOutputLeft = motorControlLeft.pidVelocity(encoderLeft, wheelSetPoints[0]);
        double motorOutputRight = motorControlRight.pidVelocity(encoderRight, wheelSetPoints[1]);
//        log("Sleft:" + wheelSetPoints[0] + " Sright:" + wheelSetPoints[1] + " Pleft:" + motorOutputLeft + " Pright:" + motorOutputRight);
        return new double[]{motorOutputLeft, motorOutputRight};
    }

    public double[] getRobotVelocities() {
//        log("Vleft: " + motorControlLeft.getDerivative() + "Vright: " + motorControlRight.getDerivative());
        return wheelsToRobot(motorControlLeft.getDerivative(), motorControlRight.getDerivative());

    }

    private double[] robotToWheels(double linear, double angular) {
        double Vleft = (linear / WHEEL_RADIUS) - (angular * WHEEL_DISTANCE) / (2 * WHEEL_RADIUS);
        double Vright = (linear / WHEEL_RADIUS) + (angular * WHEEL_DISTANCE) / (2 * WHEEL_RADIUS);
//        log(Vleft+","+Vright);
        return new double[]{Vleft, Vright};
    }

    public double[] wheelsToRobot(double Vleft, double Vright) {
        double linear = (Vright + Vleft) * WHEEL_RADIUS / 2.0;
        double angular = (Vleft - Vright) * WHEEL_RADIUS / WHEEL_DISTANCE;
        return new double[]{linear, angular};
    }

    public void updateOdometry() {
        double theta;
        theta = 0;
        double encoderLeft = left.getEncoder().getRaw() * ENCODER_TO_RADIAN;
        double encoderRight = right.getEncoder().getRaw() * ENCODER_TO_RADIAN;
//        double deltaWheel = (encoderLeft+encoderRight)*WHEEL_RADIUS/2.0;
//        double delta_x = ((((right.getEncoder().getRaw() + left.getEncoder().getRaw()) / 2.0) / 9036.0) * 0.659715);
        if (gyro == null) {
            log("gyro is null");
//            theta = thetaRobotPrev + (thetaRight - thetaRightPrev - thetaLeft + thetaLeftPrev) * (WHEEL_RADIUS / WHEEL_DISTANCE);
        } else {
            theta = Math.toRadians(gyro.getCountedAngle());
        }
//        log(Double.toString(gyro.countedAngle) + "," + Double.toString(gyro.getAngle()));
//        log("Rad: "+theta);
        //x += (realVOmega[0] * Math.cos(theta) * 0.02);
//        double distance = (deltaWheel - deltaWheelPrev);
        double distance = (encoderRight - rightEncPrev + encoderLeft - leftEncPrev) * WHEEL_RADIUS / 2.0;

        x += distance * Math.cos(theta);
//        y += (realVOmega[0] * Math.sin(theta) * 0.02);
//        theta = Math.toRadians(gyro.getCountedAngle());
//
        y += distance * Math.sin(theta);
//        log("x:" + x + ",y:" + y + ",theta:" + theta);
//        log("gyro: " + gyro.getCountedAngle());
        //log(""+realVOmega[0]);
//        log("rightEnc: "+right.getEncoder().getRaw()+" leftEnc: "+left.getEncoder().getRaw());
//        log("right_advance: "+ (encoderRight-rightEncPrev)*WHEEL_RADIUS+" left_advance: "+ (encoderLeft-leftEncPrev)*WHEEL_RADIUS);
        thetaRobotPrev = theta;
//        deltaWheelPrev = deltaWheel;
        rightEncPrev = encoderRight;
        leftEncPrev = encoderLeft;
//        xPrev = x;
//        yPrev = y;
        odometry.setX(x);
        odometry.setY(-y);
        odometry.setTheta(theta);
        odometry.setLinear(realVOmega[0]);
        odometry.setAngular(realVOmega[1]);
        odometry.setRightSetpoint(Rsetpoint);
        odometry.setLeftSetpoint(Lsetpoint);
    }

    public void initGyro(Gyroscope gyro) {
        this.gyro = gyro;
        this.gyro.calculateRate();
        log(Double.toString(this.gyro.offset));
    }

    public void direct(double leftSpeed, double rightSpeed) {
        left.set(leftSpeed);
        right.set(rightSpeed);
    }

    @Override
    public JSONObject toJSON() {
        JSONObject returnObject = new JSONObject();
        try {
            returnObject.put(LEFT, left.toJSON());
            returnObject.put(RIGHT, right.toJSON());
            returnObject.put(ODOMETRY, odometry.toJSON());
        } catch (Exception ignored) {
        }
        return returnObject;
    }
}
