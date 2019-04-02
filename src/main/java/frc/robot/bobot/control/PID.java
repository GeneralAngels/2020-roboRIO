package frc.robot.bobot.control;

import frc.robot.bobot.Subsystem;

public class PID extends Subsystem {
    public double kp, ki, kd, kf, measurement, measurementPrev, dt, derivative, derivativePrev, error, errorPrev, integral, setPointMin, controlSignal, tolerance, alpha, integralMax, signalMin, signalMax, minErrorIntegral, lastSetPoints;

    public PID() {
        this.kp = 1;
        this.ki = 0;
        this.kd = 0;
        this.kf = 0;
        this.dt = 0.02;
        this.measurement = 0;
        this.measurementPrev = 0;
        this.derivative = 0;
        this.derivativePrev = 0;
        this.error = 0;
        this.errorPrev = 0;
        this.integral = 0;
        this.setPointMin = 0.01;
        this.controlSignal = 0;
        this.tolerance = 0.01;
        this.alpha = 0.5;
        this.integralMax = 3;
        this.signalMin = 0;
        this.signalMax = 12;
        this.minErrorIntegral = 10;
        this.lastSetPoints = 0;
    }

    public void setMeasurement(double value) {
        measurementPrev = measurement;
        measurement = value;
    }

    public double filter(double value, double valuePrev) {
        return value * (1 - alpha) + valuePrev * alpha;
    }

    public void calcDerivative() {

        derivative = (measurement - measurementPrev) / dt;
        derivative = filter(derivative, derivativePrev);
        if (Math.abs(derivative) < 0.001) derivative = 0;
        derivativePrev = derivative;
    }

    public double constrain(double value, double minValue, double maxValue) {
        if (value > maxValue)
            value = maxValue;
        if (value < minValue)
            value = minValue;
        return value;
    }

    public double pidPosition(double mes, double setPoint) {
        calcDerivative();
        return pidPosition(mes, setPoint, derivative);
    }

    public double pidPosition(double mes, double setPoint, double derivative) {
        setMeasurement(mes);
        error = setPoint - measurement;
        if (Math.abs(error) < tolerance)
            controlSignal = 0;
        else {
            integral += ((error + errorPrev) * dt) / 2 * ki;
            integral = constrain(integral, -integralMax, integralMax);
            controlSignal = (error * kp) + integral - (derivative * kd);
        }
        errorPrev = error;
        controlSignal = constrain(controlSignal, -signalMax, signalMax);

        return controlSignal;
    }
    public double pidGravity(double setPoint,double mes,double compensation) {
        calcDerivative();
        return pidGravity(setPoint,mes,derivative,compensation);
    }
    public double pidGravity(double setpoint,double measurement,double derivative,double compensation){
        setMeasurement(measurement);
        error = setpoint - measurement;
//        log("error: "+error);
        if(Math.abs(error) < minErrorIntegral){
            integral += ((error + errorPrev) * dt) / 2 * ki;
            integral = constrain(integral, -integralMax, integralMax);
        }
        else {
            integral = 0;
        }
        controlSignal = (error * kp) + integral - (derivative * kd) + compensation;
        errorPrev = error;
        controlSignal = constrain(controlSignal, -signalMax, signalMax);
        return controlSignal;
    }
    public double pidVelocity(double mes, double setPoint) {
        setMeasurement(mes);
        calcDerivative();
        error = setPoint - derivative;
        if (Math.abs(setPoint) < setPointMin) {
            // Zeroing controlSignal prevents braking when setpoint returns from high to 0
            if (Math.abs(derivative) <= 0.01)
                controlSignal = 0;
                // Without this "else", loop might skip this condition and get stuck on a positive output.
            else
                controlSignal = (error * kp);
            integral = 0;
        } else {
            integral += ((error + errorPrev) * dt) / 2.0 * ki;
            integral = constrain(integral, -integralMax, integralMax);
            controlSignal = (setPoint * kf) + (error * kp) + integral;
        }
        errorPrev = error;
        controlSignal = constrain(controlSignal, -signalMax, signalMax);
        return controlSignal;
    }



    public void setPIDF(double kP, double kI, double kD, double kF) {
        this.kp = kP;
        this.ki = kI;
        this.kd = kD;
        this.kf = kF;
    }

    public double getDerivative() {
        calcDerivative();
        return derivative;
    }

    public double getIntegral() {
        return integral;
    }

    public void setIntegralMax(double value) {
        integralMax = value;
    }
}