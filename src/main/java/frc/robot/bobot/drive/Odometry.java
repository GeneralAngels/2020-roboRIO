package frc.robot.bobot.drive;

import frc.robot.bobot.Subsystem;
import org.json.JSONObject;

public class Odometry extends Subsystem {

    public static final String X = "x", Y = "y", THETA = "theta", LINEAR = "linear_velocity", ANGULAR = "angular_velocity", LEFT_SETPOINT = "left_setpoint", RIGHT_SETPOINT = "right_setpoint";

    private double x, y, theta, linear, angular, rsp, lsp;

    public double getRsp() {
        return rsp;
    }

    public Odometry setRsp(double rsp) {
        this.rsp = rsp;
        return this;
    }

    public double getLsp() {
        return lsp;
    }

    public Odometry setLsp(double lsp) {
        this.lsp = lsp;
        return this;
    }

    public double getX() {
        return x;
    }

    public Odometry setX(double x) {
        this.x = x;
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
        odometry.put(RIGHT_SETPOINT, getRsp());
        odometry.put(LEFT_SETPOINT, getLsp());
        odometry.put(THETA, getTheta());
        odometry.put(X, getX());
        odometry.put(Y, getY());
        odometry.put(LINEAR, getLinear());
        odometry.put(ANGULAR, getAngular());
        return odometry;
    }
}