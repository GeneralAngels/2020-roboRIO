package frc.robot.base.drive;

import com.ga2230.shleam.advanced.frc.FRCModule;
import com.ga2230.shleam.base.structure.Function;
import com.ga2230.shleam.base.structure.Result;
import frc.robot.base.control.path.Point;

import javax.annotation.Nullable;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class Odometry extends FRCModule {

    private double x, y, distance;
    private double angle, curvature;

    public Odometry() {
        super("odometry");

        register("reset", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                // Reset
                Odometry.this.reset();
                // Return success
                return Result.finished("Odometry reset");
            }
        });

        register("coordinates", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                // Gets x, y and sets the x and y
                String[] coordinates = parameter.split(",");
                if (coordinates.length == 2) {
                    // Set
                    Odometry.this.x = (Double.parseDouble(coordinates[0].trim()));
                    Odometry.this.y = (Double.parseDouble(coordinates[1].trim()));
                    // Return success
                    return Result.finished("Odometry set");
                }
                return Result.notFinished("Wrong parameters");
            }
        });

        register("angle", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                // Update angle
                Gyroscope.setAngle(Double.parseDouble(parameter));
                // Return success
                return Result.finished("Angle set");
            }
        });
    }

    public void reset() {
        // Reset all
        Odometry.this.distance = 0;
        Odometry.this.x = 0;
        Odometry.this.y = 0;
        Odometry.this.angle = 0;
        Odometry.this.curvature = 0;
        // Reset gyroscope
        Gyroscope.reset();
    }

    public void update(@Nullable double[] distanceDeltas) {
        // Update gyro values
        this.angle = Gyroscope.getAngle();
        this.curvature = Gyroscope.getAngularVelocity();
        // Make sure we can trust the deltas
        if (distanceDeltas != null && distanceDeltas.length == 2) {
            // Update encoder values
            this.distance = (distanceDeltas[0] + distanceDeltas[1]) / 2.0;
            this.x += this.distance * Math.cos(Math.toRadians(this.angle));
            this.y += this.distance * Math.sin(Math.toRadians(this.angle));
        }
        // Write to dictionary
        updateDictionary();
    }

    private void updateDictionary() {
        set("theta", String.valueOf(this.angle));
        set("omega", String.valueOf(this.curvature));
        set("distance", String.valueOf(this.distance));
        set("x", String.valueOf(this.x));
        set("y", String.valueOf(this.y));
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getAngle() {
        return this.angle;
    }

    public double getCurvature() {
        return this.curvature;
    }

    public double getDistance() {
        return distance;
    }

    public Point toPoint() {
        return new Point(x, y, angle, curvature);
    }
}