package frc.robot.base.control;

import frc.robot.base.Module;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class PID extends Module {

    private static final double DT = 0.02;

    private static final double TOLERANCE = 0.01;
    private static final double ALPHA = 0.5;
    private static final double MINIMUM_INTEGRAL = -1;
    private static final double MAXIMUM_INTEGRAL = 1;
    private static final double MINIMUM_SIGNAL = -12;
    private static final double MAXIMUM_SIGNAL = 12;
    private static final double MINIMUM_SETPOINT = 0.01;
    private static final double MINIMUM_ERROR_INTEGRAL = 10;

    private double kp;
    private double ki;
    private double kd;
    private double kf;

    private double measurement = 0;
    private double previousMeasurement = 0;
    private double derivative = 0;
    private double previousDerivative = 0;
    private double error = 0;
    private double previousError = 0;

    private double integral = 0;

    public PID(String id, double kp, double ki, double kd, double kf) {
        super(id);
        setPIDF(kp, ki, kd, kf);
        integral = 0;
        command("setpidf", new Command() {
            @Override
            public String execute(String s) throws Exception {
                String[] split = s.split(" ");
                if (split.length == 4) {
                    setPIDF(Double.parseDouble(split[0]), Double.parseDouble(split[1]), Double.parseDouble(split[2]), Double.parseDouble(split[3]));
                }
                return "OK";
            }
        });
    }

    public double alphaFilter(double value, double previousValue) {
        return value * (1 - ALPHA) + previousValue * ALPHA;
    }

    public double calculateDerivative() {
        derivative = (measurement - previousMeasurement) / DT;
        if (Math.abs(derivative) < 0.001)
            derivative = 0;
        return derivative;
    }
    public double calculateDerivative(double measurement, double previousMeasurement) {
        derivative = (measurement - previousMeasurement) / DT;
        if (Math.abs(derivative) < 0.001)
            derivative = 0;
        return derivative;
    }

    public double range(double value, double minimumValue, double maximumValue) {
        if (value > maximumValue)
            value = maximumValue;
        if (value < minimumValue)
            value = minimumValue;
        return value;
    }

    public double PIDPosition(double measurement, double setpoint) {
        return PIDPosition(measurement, setpoint, calculateDerivative());
    }

    public double PIDPosition(double measurement, double setpoint, double derivative) {
        double controlSignal;
        setMeasurement(measurement);
        error = setpoint - measurement;
        if (Math.abs(error) < TOLERANCE) {
            controlSignal = 0;
        } else {
            integral += ((error + previousError) * DT) / 2 * ki;
            integral = range(integral, MINIMUM_INTEGRAL, MAXIMUM_INTEGRAL);
            controlSignal = (error * kp) + integral - (derivative * kd);
        }
        previousError = error;
        return range(controlSignal, MINIMUM_SIGNAL, MAXIMUM_SIGNAL);
    }

    public double PIDGravity(double measurement, double setpoint, double compensation) {
        return PIDGravity(measurement, setpoint, calculateDerivative(), compensation);
    }

    public double PIDGravity(double measurement, double setpoint, double derivative, double compensation) {
        double controlSignal;
        setMeasurement(measurement);
        error = setpoint - measurement;
        if (Math.abs(error) < MINIMUM_ERROR_INTEGRAL) {
            integral += ((error + previousError) * DT) / 2 * ki;
            integral = range(integral, MINIMUM_INTEGRAL, MAXIMUM_INTEGRAL);
        } else {
            integral = 0;
        }
        previousError = error;
        return range((error * kp) + integral - (derivative * kd) + compensation, MINIMUM_SIGNAL, MAXIMUM_SIGNAL);
    }

    public double PIDVelocity(double measurement, double setpoint) {
        double controlSignal;
        //log("distance: "+measurement);
        setMeasurement(measurement);
        calculateDerivative();
        alphaFilter(derivative, previousDerivative);
        log("derivative: "+derivative);
        super.set("derivative: ", ""+derivative);
        error = setpoint - derivative;
        if (Math.abs(setpoint) < MINIMUM_SETPOINT) {
            // Zeroing controlSignal prevents braking when setpoint returns from high to 0
            if (Math.abs(derivative) < TOLERANCE) {
                controlSignal = 0;
                // Without this "else", loop might skip this condition and get stuck on a positive output.
            } else {
                controlSignal = (error * kp);
            }
            integral = 0;
        } else {
            integral += (((error + previousError) * DT) / 2.0) * ki;
            integral = range(integral, -MAXIMUM_INTEGRAL, MAXIMUM_INTEGRAL);
            controlSignal = (setpoint * kf) + (error * kp) + integral - (derivative*0); //kd???
//            log("\n\n\nvelocity: "+derivative+"\n\n\n");
        }
        previousError = error;
        previousDerivative = derivative;
        return range(controlSignal, -MAXIMUM_SIGNAL, MAXIMUM_SIGNAL);
    }

    public void setMeasurement(double value) {
        previousMeasurement = measurement;
        measurement = value;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public double getDerivative(){
        return derivative;
    }
}