package frc.robot.base.drive;

import frc.robot.base.Module;
import org.json.JSONObject;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class Odometry extends Module {
    public Odometry() {
        super("odometry");
    }

    public void setRightSetpoint(double rightSetpoint) {
        set("setpoint-right", String.valueOf(rightSetpoint));
    }

    public void setLeftSetpoint(double leftSetpoint) {
        set("setpoint-left", String.valueOf(leftSetpoint));
    }

    public void setDistance(double distance) {
        set("distance", String.valueOf(distance));
    }

    public void setX(double x) {
        set("x", String.valueOf(x));
    }

    public void setY(double y) {
        set("y", String.valueOf(y));
    }

    public void setTheta(double theta) {
        set("theta", String.valueOf(theta));
    }

    public void setOmega(double omega) {
        set("omega", String.valueOf(omega));
    }

    public double getX() {
        return Double.parseDouble(get("x"));
    }

    public double getY() {
        return Double.parseDouble(get("y"));
    }

    public double getTheta() {
        return Double.parseDouble(get("theta"));
    }

    public double getOmega() {
        return Double.parseDouble(get("omega"));
    }
}