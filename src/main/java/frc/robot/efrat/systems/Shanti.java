package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.control.PID;
import frc.robot.bobot.utils.PinMan;
import org.json.JSONObject;

public class Shanti extends Subsystem {
    public static final double DISTANCE = 0; //TODO: check value
    public static final double MIN_POT_VALUE = 4.58;  //TODO: check value
    public static final double MAX_POT_VALUE = 4.37;  //TODO: check value
    public static final double POT_110_NEG_ANGLE = 4.58;  //TODO: check value
    //-110
    public static final double POT_95_ANGLE = 4.38;  //TODO: check value
    private static final String LOCATION = "location";
    private static final String TARGET = "target_location";
    private static final String LIFT = "lift";
    private static final String STICK = "stick";
    private static final double RADIUS = 0.0185;
    private static final double TICKS_PER_REVOLUTIONS = 1024;
    private static final double ENC_TO_METERS = (2 * 3.14 * RADIUS) / (4 * TICKS_PER_REVOLUTIONS);
    //95
    private static final double LIMIT_V = 0.1;
    private static final double LIMIT_OMEGA = 0.1;
    private static final double GRAVITY_POWER_RADIUS = -0.12;
    private static final double GRAVITY_POWER_BETA = 0.15;
    private static Shanti latest;
    private double measurementPrev = 0;
    private double currentAngle = 0, targetAngleRadians = 0;
    private AnalogInput potentiometer;
    private PID liftMotor1PID, liftMotor2PID;
    private DigitalInput frontReset, backReset;
    private DigitalInput upReset, downReset;
    private WPI_TalonSRX liftMotor1, liftMotor2, stickMotor;
    private PID radiusPID;
    private PID betaPID;
    private PID stickMotorPID;
    private double radiusPrev = 0;
    private double betaPrev = 0;
    private double betaSetPoint = 0;
    private double outputPrev = 0;
    private double currentR = 0;
    private double currentBeta = 0;
    private double motorOutputBetaPrev = 0;



    public Shanti() {
        latest = this;
        stickMotor = new WPI_TalonSRX(17);
        downReset = new DigitalInput(PinMan.getNavDIO(6));
        upReset = new DigitalInput(PinMan.getNavDIO(7));
        frontReset = new DigitalInput(PinMan.getNavDIO(8));
        backReset = new DigitalInput(PinMan.getNavDIO(9));

        liftMotor1 = new WPI_TalonSRX(15);
        liftMotor1.setInverted(true);
        liftMotor2 = new WPI_TalonSRX(16);
        liftMotor2.setInverted(true);
        potentiometer = new AnalogInput(3);
        liftMotor1PID = new PID();
        liftMotor1PID.setPIDF(3, 0, 0, 0);
        liftMotor2PID = new PID();
        liftMotor2PID.setPIDF(3, 0, 0, 0);
        radiusPID = new PID();
        radiusPID.setPIDF(1, 0.7, 0, 0);
        radiusPID.minErrorIntegral = 0.1;
        betaPID = new PID();
        betaPID.setPIDF(0.3, 0.12, 0.05, 0);
        betaPID.minErrorIntegral = 0.15;
    }

    // TODO: add toJSON() to send real x and real y
    public static Shanti getInstance() {
        return latest;
    }

    public void moveToExchange() {

    }

    public boolean isAtExchange() {
        return true;//TODO remove this faked thing
    }

    public int sign(double x) {
        if (x >= 0)
            return 1;
        else if (x < 0) return -1;
        return 0;
    }

    public void set(double x, double y) {
        double[] rb = xy2rb(x, y);
        currentR = (stickMotor.getSelectedSensorPosition() * ENC_TO_METERS) + 1.02;
        currentBeta = mapValues(potentiometer.getVoltage());
        double compensationBeta = GRAVITY_POWER_BETA *(11.45/DriverStation.getInstance().getBatteryVoltage())*(currentR)*Math.cos(currentBeta);
        double compensationRadius = GRAVITY_POWER_RADIUS *(12.5/DriverStation.getInstance().getBatteryVoltage())*(currentR)*Math.sin(currentBeta);
        double motorOutputBeta = controlBeta(rb[1], currentBeta, compensationBeta);
        double motorOutputRadius = controlRadius(rb[0], currentR, compensationRadius);
        stickMotor.set(motorOutputRadius);
        liftMotor1.set(motorOutputBeta);
        liftMotor2.set(motorOutputBeta);
        motorOutputBetaPrev = motorOutputBeta;
    }

    public double[] xy2rb(double x, double y) {
        double radius = Math.sqrt((Math.pow(x, 2) + Math.pow(y, 2)));

        double beta = Math.atan2(y, x);
        return new double[]{radius, beta};
    }


    public double controlRadius(double setpointRadius, double currentR, double compensation) {
        double output = radiusPID.pidGravity(currentR,setpointRadius, compensation);
        return output;
    }

    public double controlBeta(double setpointBeta, double currentBeta, double compensation) {
        double output = 0;
        output = betaPID.pidGravity(setpointBeta,currentBeta, compensation);
        return output;
    }

    public double mapValues(double measurement, double minInput, double maxInput, double minOutput, double maxOutput) {
        double oldRange=(maxInput-minInput);
        double newRange=(maxOutput-minOutput);
        return (((measurement-minInput)*newRange)/oldRange)+minOutput;
    }


    public double mapValues(double measurement) {
        double alpha = 0.1;
        double meas = (measurement * alpha + measurementPrev * (1 - alpha));
        measurementPrev = meas;
        meas = 58.3*meas  -115;
        return Math.toRadians(meas);
    }

    private void calculateLocation() {
        // TODO Calculate "location"
    }

    public void print() {
//        log("meters: " + (stickMotor.getSelectedSensorPosition() * ENC_TO_METERS + 1.02));
        log("degrees: " + Math.toDegrees(mapValues(potentiometer.getVoltage())));
    }

    public void setStick(double speed) {
        speed /= 3;
        stickMotor.set(speed);
    }

    @Override
    public JSONObject toJSON() {
        JSONObject object = super.toJSON();
        JSONObject lift = new JSONObject();
        JSONObject stick = new JSONObject();
        stick.put(LOCATION, 0);
        stick.put(TARGET, 0);
        // TODO add lift things such as target angle and current angle
        object.put(LIFT, lift);
        object.put(STICK, stick);
        return object;
    }

    public void loopLift() {
        double motor1Velocity = liftMotor1PID.pidVelocity(currentAngle, targetAngleRadians);
        double motor2Velocity = liftMotor2PID.pidVelocity(currentAngle, targetAngleRadians);

    }

    public void setLift(double speed) {
        // why /6?k
//        speed /= 6;
        liftMotor1.set(speed);
        liftMotor2.set(speed);
    }
}