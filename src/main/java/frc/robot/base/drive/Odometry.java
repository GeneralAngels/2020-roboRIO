package frc.robot.base.drive;

import frc.robot.base.Module;
import org.json.JSONObject;

public class Odometry extends Module {

    public static final String X = "x", Y = "y", THETA = "theta", LINEAR = "linear_velocity", ANGULAR = "angular_velocity", LEFT_SETPOINT = "left_setpoint", RIGHT_SETPOINT = "right_setpoint";

    private double x, y, theta, linear, angular, rightSetpoint, leftSetpoint, distance;

    public double getRightSetpoint() {
        return rightSetpoint;
    }

    public Odometry setRightSetpoint(double rightSetpoint) {
        this.rightSetpoint = rightSetpoint;
        return this;
    }

    public double getLeftSetpoint() {
        return leftSetpoint;
    }

    public Odometry setLeftSetpoint(double leftSetpoint) {
        this.leftSetpoint = leftSetpoint;
        return this;
    }

    public double getX() {
        return x;
    }

    public Odometry setX(double x) {
        this.x = x;
        return this;
    }

    public double getDistance() {
        return distance;
    }

    public Odometry setDistance(double distance) {
        this.distance = distance;
        return this;
    }

    public double getLinear() {
        return linear;
    }

    public Odometry setLinear(double linear) {
        this.linear = linear;
        return this;
    }

    public double getAngular() {
        return angular;
    }

    public Odometry setAngular(double angular) {
        this.angular = angular;
        return this;
    }

    public double getY() {
        return y;
    }

    public Odometry setY(double y) {
        this.y = y;
        return this;
    }

    public double getTheta() {
        return theta;
    }

    public Odometry setTheta(double theta) {
        this.theta = theta;
        return this;
    }

    @Override
    public JSONObject toJSON() {
        JSONObject odometry = new JSONObject();
        odometry.put(RIGHT_SETPOINT, getRightSetpoint());
        odometry.put(LEFT_SETPOINT, getLeftSetpoint());
        odometry.put(THETA, getTheta());
        odometry.put(X, getX());
        odometry.put(Y, getY());
        odometry.put(LINEAR, getLinear());
        odometry.put(ANGULAR, getAngular());
        odometry.put("distance", distance);
        return odometry;
    }
}