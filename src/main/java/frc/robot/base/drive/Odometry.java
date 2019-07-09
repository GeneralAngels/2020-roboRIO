package frc.robot.base.drive;

import frc.robot.base.Module;
import org.json.JSONObject;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class Odometry extends Module {

    private double x, y, theta, linear, angular, rightSetpoint, leftSetpoint, distance;

    public double getRightSetpoint() {
        return rightSetpoint;
    }

    public void setRightSetpoint(double rightSetpoint) {
        this.rightSetpoint = rightSetpoint;
        log("setpoint-right", rightSetpoint);
    }

    public double getLeftSetpoint() {
        return leftSetpoint;
    }

    public void setLeftSetpoint(double leftSetpoint) {
        this.leftSetpoint = leftSetpoint;
        log("setpoint-left", leftSetpoint);
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
        log("distance", distance);
    }

    public double getLinear() {
        return linear;
    }

    public void setLinear(double linear) {
        this.linear = linear;
        log("linear", linear);
    }

    public double getAngular() {
        return angular;
    }

    public void setAngular(double angular) {
        this.angular = angular;
        log("angular", angular);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
        log("x", x);
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
        log("y", y);
    }

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
        log("theta", theta);
    }
}