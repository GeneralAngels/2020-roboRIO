package frc.robot.base.control.path;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.utils.MotorGroup;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;

public class PathFollower extends frc.robot.base.Module {
    public PathFollower(DifferentialDrive drive) {
        super("follower");
        command("createpath", new Command() {
            @Override
            public String execute(String s) throws Exception {
                String[] split = s.split(" ");
                if (split.length == 5) {
                    drive.setTrajectory(createPath());
                }
                return "OK";
            }

//            command("visionpath",new Command() {
//                @Override
//                public String execute(String s) throws Exception {
//                    String[] split = s.split(" ");
//                    if (split.length == 5) {
//                        drive.setTrajectory(visionpath());
//                    }
//                    return "OK";
//                }
        });
        command("get_da_yeet", new Command() {
            @Override
            public String execute(String s) throws Exception {
                if (trajectory != null) {
                    JSONArray array = new JSONArray();
                    for (Trajectory.State state : trajectory.getStates()) {
                        JSONObject object = new JSONObject();
                        object.put("x", state.poseMeters.getTranslation().getX());
                        object.put("y", state.poseMeters.getTranslation().getY());
                        object.put("angle", state.poseMeters.getRotation().getDegrees());
                        array.put(object);
                    }
                    return array.toString();
                }
                return "[]";
            }
        });
    }

    public double[][] getPoints(double[] a, double[] b, int amount) {//coefx, coefy, amount of point
        double[][] points = new double[amount][2];

        for (int i = 0; i < amount; i++) {
            points[i][0] = put(a, ((double) i) / amount);
            points[i][1] = put(b, ((double) i) / amount);
        }
        return points;
    }

    private double put(double[] a, double val) {
        double ret = 0;
        for (int i = 0; i < a.length; i++) {
            ret += a[i] * Math.pow(val, a.length - i - 1);
        }
        return ret;
    }

    public Trajectory createPath() {
        //we don't need the abcdk. the library does this itself

        Pose2d startPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d endPoint = new Pose2d(1, 0, Rotation2d.fromDegrees(-90));
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
        //interiorWaypoints.add(new Translation2d(1, 0));
//        interiorWaypoints.add(new Translation2d(1.0, 0.5));

        TrajectoryConfig config = new TrajectoryConfig(4, 1);
        config.setEndVelocity(0);
        //config.setReversed(true);
        trajectory = TrajectoryGenerator.generateTrajectory(startPoint, interiorWaypoints, endPoint, config);
        trajectory.getStates();
        return trajectory;

    }

    private Trajectory trajectory;

//    static class TrajectoryGenerator {
//        static double[] get_force(double gyro, double encoderLeft, double encoderRight, double wheel_dist, double wheel_diameter) {
//            ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
//            double time_step = 0.02;
//            double max_velocity = 1.7;
//            double max_acceleration = 2;
//            double max_jerk = 60;
//
//            Tragectory.Config.config = new Tragectory.Config(Tragectory.FitMethod.HERMITE_CUBIC, Tragectory.Config.SAMPLES_HIGH, time_step, max_velocity, max_acceleration, max_jerk);
//
//            Tragectory tragectory = Pathfinder.generate(points, config);
//            TankModifier modifier = new TankModifier(tragectory).modify(wheel_dist);
//            EncoderFollower left = new EncoderFollower(modifier.getLeftTragectory());
//            EncoderFollower right = new EncoderFollower(modifier.getRightTragectory());
//
//            double tpr = 2048;
//            right.configureEncoder(encoderRight, tpr, wheel_diameter);
//            left.configureEncoder(encoderLeft, tpr, wheel_diameter);
//            double kp = 1, ki = 0, kd = 0.1, kv = 1 / max_velocity, ka = 0;
//            right.configurePIDVA(kp, ki, kd, kv, ka);
//            left.configurePIDVA(kp, ki, kd, kv, ka);
//
//            double left_power = 0, right_power = 0;
//            left_power += left.calculate(encoderLeft);
//            right_power += right.calculate(encoderRight);
//            //TODO: the gyro thing
//        }
//    }
}
