package frc.robot.base.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.base.Module;
import frc.robot.base.control.PID;
import frc.robot.base.utils.MotorGroup;
import frc.robot.base.utils.StickDrive;
import org.json.JSONObject;

// TODO redo the whole thing
// TODO once complete, add copyright comment, its too embarrassing rn

public class DifferentialDrive<T extends SpeedController> extends Module {
    public double Lsetpoint = 0;
    public double Rsetpoint = 0;
    public PID motorControlLeft;
    public PID motorControlRight;
    public PID motorControlLeftP;
    public PID motorControlRightP;
    public AHRS gyro;
    public double WHEEL_DISTANCE = 0.51;
    public double WHEEL_RADIUS = 0.1016;
    public double MAX_V = 2;

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
    public double[] vOmega = {0, 0};
    public double offsetGyro = 0;
    public double distanceFromEncoders = 0;
    public double theta = 0;
    public double motorOutputLeft = 0;
    public double motorOutputRight = 0;
    public boolean check = true;
    public double currentMetersRight = 0;
    public double currentMetersLeft = 0;
    public PID pid;
    protected MotorGroup<T> left = new MotorGroup<>(), right = new MotorGroup<>();
    protected Odometry odometry = new Odometry();
    double motorOutputLeftPrev = 0;
    double motorOutputRightPrev = 0;
    double battery = 0;
    double batteryPrev = 0;
    boolean checkGyro = true;

    public DifferentialDrive() {
        motorControlLeft = new PID();
        motorControlRight = new PID();
        motorControlLeftP = new PID();
        motorControlRightP = new PID();
        pid = new PID();
        motorControlLeft.setPIDF(0, 0, 0, 0.55);
        motorControlRight.setPIDF(0, 0, 0, 0.55);
        motorControlLeftP.setPIDF(4, 0.47, 0, 0);
        motorControlRightP.setPIDF(4, 0.8, 0, 0);
        pid.setPIDF(1, 0.1, 0, 0);
        register(odometry);
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

    public void setAutonomous(double v, double w, boolean auto) {
        set(v, w, auto);
    }

    public void setStickNoPID(double speed, double turn) {
        direct(StickDrive.left(speed, turn), StickDrive.right(speed, turn));
    }

    public void setTank(double Vl, double Vr) {
        if (Math.abs(Vl) < 0.1)
            Vl = 0;
        if (Math.abs(Vr) < 0.1)
            Vr = 0;
        direct(-Vl, -Vr);
        updateOdometry();
    }

    public void setTank2(double Vl, double Vr) {
        log("JoystickL", Vl);
        log("JoystickR", Vr);
        if (Math.abs(Vl) < 0.15) {
            Vl = 0;
        }
        if (Math.abs(Vr) < 0.15) {
            Vr = 0;
        }
        double vSetPoint = (Vl + Vr) / 2.0;
        double omegaSetPoint = (Vr - Vl) / 2.0;
        set(vSetPoint * MAX_V, omegaSetPoint * MAX_OMEGA, false);
    }

    public void set(double speed, double turn, boolean auto) {
        if (auto) {
            motorControlLeft.setPIDF(0.5, 0.3, 0, 0.5);
            motorControlRight.setPIDF(0.5, 0.3, 0, 0.5);
            log("yes");
        } else {
            motorControlLeft.setPIDF(0, 0, 0, 0.55);
            motorControlRight.setPIDF(0, 0, 0, 0.55);
            log("no");
        }

        double setpointV = speed;
        double setpointOmega = turn;
        double[] V = robotToWheels(setpointV, setpointOmega);
        log("applyPower point v", setpointV);
        log("applyPower point omega", setpointOmega);
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
        battery = DriverStation.getInstance().getBatteryVoltage();
        battery = (0.5 * battery) + (0.5 * batteryPrev);
        if (battery > 12.0)
            battery = 12.0;
        batteryPrev = battery;
        motorOutputLeftPrev = motorOutputLeft;
        motorOutputRightPrev = motorOutputRight;
        setPointVPrev = setpointV;
        setPointOmegaPrev = setpointOmega;
        log("Batt", battery);
        log("DirectL", -(motorOutputLeft / battery));
        log("DirectR", -(motorOutputRight / battery));
        direct(-(motorOutputLeft / battery), -(motorOutputRight / battery));
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
        double angular = (Vright - Vleft) * WHEEL_RADIUS / WHEEL_DISTANCE;
        return new double[]{linear, angular};
    }

    public void updateOdometry() {
        theta = 0;
        if (gyro == null) {
            log("gyro", "Not Working");
        } else {
            if (checkGyro) {
                checkGyro = false;
                gyro.reset();
            } else {
                log("gyro", "Working");
                theta = gyro.getYaw();
            }
        }
        leftMeters = (encoders[0] - encodersPrev[0]) * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
        rightMeters = (encoders[1] - encodersPrev[1]) * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
        VOmegaReal = wheelsToRobot(motorControlLeft.derivative, motorControlRight.derivative);
        distanceFromEncoders = (leftMeters + rightMeters) / 2.0;
        x += distanceFromEncoders * Math.cos(toRadians(theta));
        y += distanceFromEncoders * Math.sin(toRadians(theta));
        thetaRobotPrev = theta;
        encodersPrev[0] = encoders[0];
        encodersPrev[1] = encoders[1];
        odometry.setX(x);
        odometry.setY(-y);
        odometry.setTheta(theta);
        odometry.setRightSetpoint(Rsetpoint);
        odometry.setLeftSetpoint(Lsetpoint);
        odometry.setDistance(distanceFromEncoders);
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
        direct(-(motorOutputLeft), -(motorOutputRight));
        leftMeters = left.getEncoder().getRaw() * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
        rightMeters = right.getEncoder().getRaw() * gearRatio * ENCODER_TO_RADIAN * WHEEL_RADIUS;
    }

    public void initGyro(AHRS gyro) {
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

    @Override
    public JSONObject pullJSON() {
        log("v_robot_real", VOmegaReal[0]);
        log("omega_robot_real", VOmegaReal[1]);
        log("left_encoder", encoders[0]);
        log("right_encoder", encoders[1]);
        log("v left real:", motorControlLeft.derivative);
        log("v right real:", motorControlRight.derivative);
        return super.pullJSON();
    }
}
