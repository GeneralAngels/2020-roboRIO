package frc.robot.base.control.path;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.base.drive.DifferentialDrive;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

public class PathManager {

    private static Trajectory trajectory;

    public double[][] getPoints(double[] a, double[] b, int amount) {//coefx, coefy, amount of point
        double[][] points = new double[3][amount];

        for (int i = 0; i < amount; i++) {
            points[i][0] = put(a, ((double) i) / amount);
            points[i][1] = put(b, ((double) i) / amount);
            //points[i][2] = Math.atan(put())
        }
        return points;
    }

    public double put(double[] a, double val) {
        double ret = 0;
        for (int i = 0; i < a.length; i++) {
            ret += a[i] * Math.pow(val, a.length - i - 1);
        }
        return ret;
    }

    public static Trajectory createPath(ArrayList<Translation2d> waypoints) {

        Pose2d startPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d endPoint = new Pose2d(2, 0.5, Rotation2d.fromDegrees(0));
        //interiorWaypoints.add(new Translation2d(1, 0));
//        interiorWaypoints.add(new Translation2d(1.0, 0.5));

        TrajectoryConfig config = new TrajectoryConfig(4, 1);
        config.setEndVelocity(0);
        //config.setReversed(true);
        trajectory = TrajectoryGenerator.generateTrajectory(startPoint, waypoints, endPoint, config);
        trajectory.getStates();
        return trajectory;
    }

    public static double movingAverageCurvature(List<Trajectory.State> trajectory) {
        int size = trajectory.size();
        double average = (Math.abs(trajectory.get(size - 1).curvatureRadPerMeter) + Math.abs(trajectory.get(size - 2).curvatureRadPerMeter)) / 2;
        for (int i = size - 3; i >= 0; --i) {
            average = (average + Math.abs(trajectory.get(i).curvatureRadPerMeter)) / 2;
        }
        return average;
    }

    public static double[] derivePolynomial(double[] coefficients) {
        double[] result = new double[coefficients.length];
        result[0] = 0;
        for (int i = 1; i < coefficients.length; i++) {
            result[i] = coefficients[i - 1] * i;
        }
        return result;
    }

    public static Trajectory getTrajectory() {
        return trajectory;
    }
}
