package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.control.PID;
import frc.robot.bobot.utils.PinManager;
import org.json.JSONObject;

public class Shanti extends Subsystem {
    public static final double MAX_ANGLE_RADIANS = 1;
    public static final double[] ROCKET_ANGLES_RADIANS = {0, 0.5, 0.9};
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
    private static final double STICK_LENGTH_METERS = 1.22;
    private static final double RADIUS = 0.0185;
    private static final double TICKS_PER_REVOLUTIONS = 1024;
    private static final double ENC_TO_METERS = (2 * 3.14 * RADIUS) / (4 * TICKS_PER_REVOLUTIONS);
    private static Shanti latest;
    //95
    private double measurementPrev = 0;
    private AnalogInput potentiometer;
    private PID liftMotor1PID, liftMotor2PID;
    private double location = 0;
    private double targetLocation = 0;
    private DigitalInput startReset, endReset;
    private Encoder encoder;
    private double currentAngle = 0;
    private int targetLevel = 0;
    private double targetAngleRadians = 0;
    private DigitalInput upReset, downReset;
    private WPI_TalonSRX liftMotor1, liftMotor2, stickMotor;
    private PID radiusPID;
    private PID betaPID;


    public Shanti() {
        latest = this;
        PinManager pinManager = new PinManager();
        stickMotor = new WPI_TalonSRX(17);
//        encoder = new Encoder(8, 9);
//        stickMotor.getSelectedSensorPosition(); // how to get encoder
//        frontReset = new DigitalInput(2);
//        endReset = new DigitalInput(3);

        liftMotor1 = new WPI_TalonSRX(15);
        liftMotor2 = new WPI_TalonSRX(16);
        potentiometer = new AnalogInput(2);
        downReset = new DigitalInput(0);
        upReset = new DigitalInput(1);
        liftMotor1PID = new PID();
        liftMotor1PID.setPIDF(0.7, 0, 0, 0);
        liftMotor2PID = new PID();
        liftMotor2PID.setPIDF(0.7, 0, 0, 0);
        radiusPID = new PID();
        radiusPID.setPIDF(0.7, 0, 0, 0);
        betaPID = new PID();
        betaPID.setPIDF(0.1, 0, 0, 0);
    }

    // TODO: add toTSON() to send real x and real y
    public static Shanti getInstance() {
        return latest;
    }

    public void moveToExchange() {

    }

    public boolean isAtExchange() {
        return true;//TODO remove this faked thing
    }

    public void set(double x, double y) {
        double[] rb = xy2rb(x, y);
        double motorOutputRadius = controlRadius(rb[0]);
//        log("radius:" +Double.toString(motorOutputRadius));
//        double motorOutputBeta = controlBeta(rb[1]);
        log("motor:" + motorOutputRadius);
        stickMotor.set(motorOutputRadius);
//        liftMotor1.set(motorOutputBeta);
//        liftMotor2.set(motorOutputBeta);
    }

    public double[] xy2rb(double x, double y) {
        double radius = Math.sqrt(((x * x) + (y * y)));
        double beta = Math.atan2(y, x);
        return new double[]{radius, beta};
    }

    public double controlRadius(double setpointRadius) {
        double currentRadius = stickMotor.getSelectedSensorPosition() * ENC_TO_METERS;
//        log(Double.toString(setpointRadius - currentRadius));
        double output = radiusPID.pidPosition(currentRadius, setpointRadius);
        return output;
    }

    public double controlBeta(double setpointBeta) {
        double currentBeta = mapValues(potentiometer.getVoltage()); //TODO: check getVoltage()
        double output = betaPID.pidPosition(currentBeta, setpointBeta);
        return output;
    }

    public double mapValues(double measurement, double minInput, double maxInput, double minOutput, double maxOutput) {
        return (measurement - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput;
    }

    public double mapValues(double measurement) {
        measurement = (measurement * 1000 - (measurement * 1000 % 1));
        double meas = measurement;
        if (-10 < (measurement - measurementPrev) && (measurement - measurementPrev) < 10
        ) {
            meas = measurementPrev;
//            log("yes");
        }
        // log("mes: "+ meas);
        double minInput = POT_110_NEG_ANGLE * 100; // 4.58
        double maxInput = POT_95_ANGLE * 100; // 4.38
        double minOutput = -88;
        double maxOutput = 95;
        // double minOutput = MIN_POT_VALUE;
        // double maxOutput = MAX_POT_VALUE;
        measurementPrev = meas;
        meas = (int) meas / 10;
        return Math.toRadians((meas - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput);
    }

    private void calculateLocation() {
        // TODO Calculate "location"
    }

    public void print() {
        log("meters: " + stickMotor.getSelectedSensorPosition() * ENC_TO_METERS);
//        log("potentiometer voltage: "+potentiometer.getVoltage());
//        log("degrees: "+ mapValues(potentiometer.getVoltage()));
    }

    public void setTargetLocation(double targetLocation) {
        this.targetLocation = targetLocation;
    }

    public void loopStick() {
        if (startReset != null) {
            if (!startReset.get()) {
                // Pressed
                location = 0;
            }
        }
        if (startReset != null) {
            if (!endReset.get()) {
                // Pressed
                location = STICK_LENGTH_METERS;
            }
        }
        calculateLocation();
        // Do PID Stuff
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
        stick.put(LOCATION, location);
        stick.put(TARGET, targetLocation);
        // TODO add lift things such as target angle and current angle
        object.put(LIFT, lift);
        object.put(STICK, stick);
        return object;
    }

    public void setTargetLevel(int targetLevel) {
        if (targetLevel < ROCKET_ANGLES_RADIANS.length) {
            this.targetLevel = targetLevel;
            targetAngleRadians = ROCKET_ANGLES_RADIANS[targetLevel];
        }
    }


    public void loopLift() {
        double motor1Velocity = liftMotor1PID.pidVelocity(currentAngle, targetAngleRadians);
        double motor2Velocity = liftMotor2PID.pidVelocity(currentAngle, targetAngleRadians);

    }

    public void setLift(double speed) {
        // why /6?
        speed /= 6;
        liftMotor1.set(speed);
        liftMotor2.set(speed);
    }

    public void levelUp() {
        if (targetLevel < ROCKET_ANGLES_RADIANS.length - 1) {
            targetLevel++;
            targetAngleRadians = ROCKET_ANGLES_RADIANS[targetLevel];
        }
    }

    public void levelDown() {
        if (targetLevel > 0) {
            targetLevel--;
            targetAngleRadians = ROCKET_ANGLES_RADIANS[targetLevel];
        }
    }
}