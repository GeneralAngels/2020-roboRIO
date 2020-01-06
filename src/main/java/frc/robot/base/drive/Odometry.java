package frc.robot.base.drive;

import frc.robot.base.Module;
import org.json.JSONObject;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class Odometry extends Module {

    private double x, y, theta, linear, angular, rightSetpoint, leftSetpoint, distance;

    public Odometry() {
        super("odometry");
    }

    public double getRightSetpoint() {
        return rightSetpoint;
    }

    public void setRightSetpoint(double rightSetpoint) {
        this.rightSetpoint = rightSetpoint;
        set("setpoint-right", String.valueOf(rightSetpoint));
    }

    public double getLeftSetpoint() {
        return leftSetpoint;
    }

    public void setLeftSetpoint(double leftSetpoint) {
        this.leftSetpoint = leftSetpoint;
        set("setpoint-left", String.valueOf(leftSetpoint));
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
        set("distance", String.valueOf(distance));
    }

    public double getLinear() {
        return linear;
    }

    public void setLinear(double linear) {
        this.linear = linear;
        set("linear", String.valueOf(linear));
    }

    public double getAngular() {
        return angular;
    }

    public void setAngular(double angular) {
        this.angular = angular;
        set("angular", String.valueOf(angular));
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
        set("x", String.valueOf(x));
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
        set("y", String.valueOf(y));
    }

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
        set("theta", String.valueOf(theta));
    }
}