package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
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


    public Shanti() {
        latest = this;
        stickMotor = new WPI_TalonSRX(17);
//        stickMotor.getSelectedSensorPosition(); // how to get encoder
        potentiometer = new AnalogInput(PinMan.getNavAIN(0));
        downReset = new DigitalInput(PinMan.getNavDIO(6));
        upReset = new DigitalInput(PinMan.getNavDIO(7));
        frontReset = new DigitalInput(PinMan.getNavDIO(8));
        backReset = new DigitalInput(PinMan.getNavDIO(9));
        liftMotor1 = new WPI_TalonSRX(15);
        liftMotor2 = new WPI_TalonSRX(16);
        liftMotor1PID = new PID();
        liftMotor1PID.setPIDF(3, 0, 0, 0);
        liftMotor2PID = new PID();
        liftMotor2PID.setPIDF(3, 0, 0, 0);
        radiusPID = new PID();
        radiusPID.setPIDF(0.5, 0.1, 0.6, 0);
        betaPID = new PID();
        betaPID.setPIDF(0.6, 0.1, 0.3, 0);
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
        if (x > 0)
            return 1;
        else if (x < 0) return -1;
        return 0;
    }

    public void set(double x, double y) {
        double[] rb = xy2rb(x, y);
        double motorOutputRadius = controlRadius(rb[0]);
//        log("radius:" +Double.toString(motorOutputRadius));
//        double motorOutputBeta = controlBeta(rb[1]);
        log("motor:" + motorOutputRadius);
        double motorOutputBeta = -controlBeta(rb[1]);
        log("motor:" + motorOutputRadius);
        stickMotor.set(motorOutputRadius);
        liftMotor1.set(motorOutputBeta);
        liftMotor2.set(motorOutputBeta);
//        liftMotor1.set(motorOutputBeta);
//        liftMotor2.set(motorOutputBeta);
    }

    public double[] xy2rb(double x, double y) {
        double radius = Math.sqrt((Math.pow(x, 2) + Math.pow(y, 2)));
        double beta = Math.atan2(y, x);
        return new double[]{radius, beta};
    }


    public double controlRadius(double setpointRadius) {
        double currentRadius = stickMotor.getSelectedSensorPosition() * ENC_TO_METERS + 0.5;
//        log(Double.toString(setpointRadius - currentRadius));
        double output = radiusPID.pidPosition(currentRadius, setpointRadius);
        return output;
    }

    public double controlBeta(double setpointBeta) {
        double currentBeta = mapValues(potentiometer.getVoltage());
        double output = 0;
        log("error beta: " + (setpointBeta - currentBeta));
//        if(Math.abs((currentBeta-betaPrev)/0.02) > LIMIT_OMEGA)
//            output = LIMIT_OMEGA * sign(currentBeta-betaPrev);
//        else
        output = betaPID.pidPosition(currentBeta, setpointBeta);
        double alpha = 0.005;
        output = (output * alpha) + (outputPrev * (1 - alpha));
        betaPrev = currentBeta;
        outputPrev = output;
        return output;
    }

    public double mapValues(double measurement, double minInput, double maxInput, double minOutput, double maxOutput) {
        return (measurement - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput;
    }

    public double mapValues(double measurement) {
        double alpha = 0.1;
        double meas = (measurement * alpha + measurementPrev * (1 - alpha));
        measurementPrev = meas;
        if (Math.abs(-383 * meas + 1839) >= 90)
            return betaSetPoint;
        return Math.toRadians(-383 * meas + 1839);
//        measurement = (measurement*1000- (measurement*1000%1));
//        double meas = measurement;
//        if(-10<(measurement-measurementPrev) && (measurement-measurementPrev)<10) {
//            meas = measurementPrev;
//////            log("yes");
//        }
//       // log("mes: "+ meas);
//        double minInput = POT_110_NEG_ANGLE*1000; // 4.58
//        double maxInput = POT_95_ANGLE*1000; // 4.38
//        double minOutput = -88;
//        double maxOutput = 95;
//       // double minOutput = MIN_POT_VALUE;
//       // double maxOutput = MAX_POT_VALUE;
//       // meas = (int)meas/10;
//        meas = (int)(measurement*0.5 + measurementPrev*0.5);
//        measurementPrev = meas;
//        return Math.toRadians((meas - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput);
    }

    private void calculateLocation() {
        // TODO Calculate "location"
    }

    public void print() {
        log("meters: " + stickMotor.getSelectedSensorPosition() * ENC_TO_METERS);
        log("degrees: " + Math.toDegrees(mapValues(potentiometer.getVoltage())));

//        log("potentiometer voltage: "+potentiometer.getVoltage());
//        log("degrees: "+ mapValues(potentiometer.getVoltage()));
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
        speed /= 6;
        liftMotor1.set(speed);
        liftMotor2.set(speed);
    }
}