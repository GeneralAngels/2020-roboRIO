package frc.robot.base.control.path;

import org.json.JSONObject;

public class Point {
    private double x, y;
    private double angle, curvature;

    public Point(double x, double y, double angle, double curvature) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.curvature = curvature;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return angle;
    }

    public double getCurvature() {
        return curvature;
    }

    public JSONObject toJSON() {
        JSONObject object = new JSONObject();
        object.put("x", x);
        object.put("y", y);
        object.put("angle", angle);
        object.put("curvature", curvature);
        return object;
    }
}
