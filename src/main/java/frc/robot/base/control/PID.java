package frc.robot.base.control;

import com.ga2230.shleam.advanced.frc.FRCModule;
import com.ga2230.shleam.base.structure.Function;
import com.ga2230.shleam.base.structure.Result;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class PID extends FRCModule {


    private static final double TOLERANCE = 0.01;
    private static final double ALPHA = 0.5;
    private static final double MINIMUM_INTEGRAL = -1;
    private static final double MAXIMUM_INTEGRAL = 1;
    private static final double MINIMUM_SIGNAL = -12;
    private static final double MAXIMUM_SIGNAL = 12;
    private static final double MINIMUM_SETPOINT = 0.01;
    private static final double MINIMUM_ERROR_INTEGRAL = 10;

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private double previousTime = millis();

    private double timeDelta = 0.02;

    private double measurement = 0;
    private double previousMeasurement = 0;
    private double derivative = 0;
    private double previousDerivative = 0;
    private double error = 0;
    private double previousError = 0;

    private double integral = 0;

    public PID(String id, double kP, double kI, double kD, double kF) {
        super(id);

        setPIDF(kP, kI, kD, kF);

        register("set", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                String[] split = parameter.split(" ");
                if (split.length == 4) {
                    setPIDF(Double.parseDouble(split[0]), Double.parseDouble(split[1]), Double.parseDouble(split[2]), Double.parseDouble(split[3]));
                }
                return Result.finished("OK");
            }
        });
    }

    public double alphaFilter(double value, double previousValue) {
        return value * (1 - ALPHA) + previousValue * ALPHA;
    }

    public double calculateDerivative() {
        if (measurement == previousMeasurement)
            return derivative;
        derivative = (measurement - previousMeasurement) / timeDelta;
        if (Math.abs(derivative) < 0.001)
            derivative = 0;
        return derivative;
    }

    public double calculateDerivative(double measurement, double previousMeasurement) {
        derivative = (measurement - previousMeasurement) / timeDelta;
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
            integral += ((error + previousError) * timeDelta) / 2 * kI;
            integral = range(integral, MINIMUM_INTEGRAL, MAXIMUM_INTEGRAL);
            controlSignal = (error * kP) + integral - (derivative * kD);
        }
        previousError = error;
        return range(controlSignal, MINIMUM_SIGNAL, MAXIMUM_SIGNAL);
    }

    public double PIDGravity(double measurement, double setpoint, double compensation) {
        return PIDGravity(measurement, setpoint, calculateDerivative(), compensation);
    }

    public double PIDGravity(double measurement, double setpoint, double derivative, double compensation) {
        setMeasurement(measurement);
        error = setpoint - measurement;
        if (Math.abs(error) < MINIMUM_ERROR_INTEGRAL) {
            integral += ((error + previousError) * timeDelta) / 2 * kI;
            integral = range(integral, MINIMUM_INTEGRAL, MAXIMUM_INTEGRAL);
        } else {
            integral = 0;
        }
        previousError = error;
        return range((error * kP) + integral - (derivative * kD) + compensation, MINIMUM_SIGNAL, MAXIMUM_SIGNAL);
    }

    public void updateDelta() {
        this.timeDelta = (millis() - previousTime) / 1000;
        previousTime = millis();
    }

    public double PIDVelocity(double measurement, double setpoint) {
        double controlSignal;
        setMeasurement(measurement);
        calculateDerivative();
        error = setpoint - derivative;
        if (Math.abs(setpoint) < MINIMUM_SETPOINT) {
            // Zeroing controlSignal prevents braking when setpoint returns from high to 0
            if (Math.abs(derivative) < TOLERANCE) {
                controlSignal = 0;
                // Without this "else", loop might skip this condition and get stuck on a positive output.
            } else {
                controlSignal = (error * kP);
            }
            integral = 0;
        } else {
            integral += (((error + previousError) * timeDelta) / 2.0) * kI;
            integral = range(integral, -MAXIMUM_INTEGRAL, MAXIMUM_INTEGRAL);
            controlSignal = (setpoint * kF) + (error * kP) + integral;
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
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.kF = kf;
        this.measurement = 0;
        this.previousMeasurement = 0;
        this.integral = 0;
        this.derivative = 0;
        this.previousDerivative = 0;
        this.error = 0;
        this.previousError = 0;
        this.previousTime = millis();
    }

    public double getDerivative() {
        return derivative;
    }

    public double getError() {
        return error;
    }
}