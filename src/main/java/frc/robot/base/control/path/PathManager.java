package frc.robot.base.control.path;

import com.ga2230.shleam.advanced.frc.FRCModule;
import com.ga2230.shleam.base.structure.Function;
import com.ga2230.shleam.base.structure.Result;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.drive.Odometry;
import frc.robot.base.utils.General;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;

public class PathManager extends FRCModule {

    private static final double TIME_DELTA = 0.02;

    private static final double RANGE_TOLERANCE = 0.2;
    private static final double ANGLE_TOLERANCE = 0.05;
    private static final double MAXIMUM_VELOCITY = 1;
    private static final double MINIMUM_VELOCITY = 0.5;

    private static final double K_THETA = 4.0;
    private static final double K_CURVATURE = 1;
    private static final double K_OMEGA = 0.1;
    private static final double K_VELOCITY = 0;

    private static final boolean LOGS_ENABLED = true;

    private DifferentialDrive drive;

    private ArrayList<Point> points;

    private int index = 0;

    public PathManager(DifferentialDrive drive) {
        super("path");
        this.drive = drive;
        this.points = new ArrayList<>();

        // Command registration for autonomous

        register("fetch", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                JSONArray array = new JSONArray();
                if (points != null) {
                    for (Point point : points) {
                        array.put(point.toJSON());
                    }
                }
                return Result.finished(array.toString());
            }
        });

        register("create", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                // Parse string into three parameters (x, y, theta)
                String[] split = parameter.split(" ");
                if (split.length == 4) {
                    double x = Double.parseDouble(split[0]);
                    double y = Double.parseDouble(split[1]);
                    double theta = Double.parseDouble(split[2]);
                    createTrajectory(new Point(x, y, theta, 0), split[3].equals("reverse"));
                    return Result.finished("Trajectory created");
                } else {
                    return Result.notFinished("Wrong number of parameters");
                }
            }
        });

        register("set", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                points = new ArrayList<>();
                JSONArray array = new JSONArray(parameter);
                for (int i = 0; i < array.length(); i++) {
                    JSONObject object = array.getJSONObject(i);
                    points.add(new Point(object.getDouble("x"), object.getDouble("y"), object.getDouble("angle"), object.getDouble("curvature")));
                }
                return Result.finished("Thank you");
            }
        });

        register("follow", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                if (points != null) {
                    boolean done = followTrajectory(parameter.equals("reverse"));
                    return Result.create(done, done ? "Done" : "Not done");
                } else {
                    return Result.notFinished("No trajectory");
                }
            }
        });
    }

    private void updateProgress() {
        set("index", String.valueOf(index));
        set("length", String.valueOf(points.size()));
    }

    private void updateOdometry() {
        drive.updateOdometry();
    }

    public void createTrajectory(Point target, boolean reversed) {
        // Reset array
        points = new ArrayList<>();
        // Reset index
        index = 1;
        // Update odometry
        updateOdometry();
        // Configure trajectory
        TrajectoryConfig config = new TrajectoryConfig(2, 1);
        config.setEndVelocity(0);
        // Points
        Point start = getCurrentPoint();
        Point end = target;
        // Poses
        trajectoryToPoints(TrajectoryGenerator.generateTrajectory(pointToState(start).poseMeters, new ArrayList<>(), pointToState(end).poseMeters, config));
    }

    public boolean followTrajectory(boolean reversed) {
        // Update progress and odometry
        updateProgress();
        updateOdometry();
        // Follow trajectory
        double currentDesiredOmega;
        double currentDesiredVelocity;
        if (index < points.size()) {
            // Calculate errors
            double[] errors = calculateErrors();
            // Calculate desired angular velocity
            currentDesiredOmega = errors[2] * K_THETA - getCurrentPoint().getCurvature() * K_OMEGA;
            // Check if last point
            //if (index == points.size() - 1) { //TODO: check if changed if works
            Point lastPoint = points.get(points.size() - 1);
            if (distance(getCurrentPoint(), lastPoint) < 0.5) {
                log("last point");
                currentDesiredVelocity = errors[0] * K_VELOCITY;
                if (General.deadband(errors[0], RANGE_TOLERANCE) == 0)
                    index++;
                // TODO these two if are meaningfully the same
            } else {
                log("not last point");
                currentDesiredVelocity = MAXIMUM_VELOCITY - Math.abs(errors[2]) * K_CURVATURE;
                if (General.deadband(errors[0], errors[1]) == 0)
                    index++;
            }
            // Make sure the signal is positive
            currentDesiredVelocity = Math.max(currentDesiredVelocity, MINIMUM_VELOCITY);
            // Multiply for reverse
            currentDesiredVelocity *= (!reversed ? 1 : -1);
            // Send command
            drive.driveVector(currentDesiredVelocity, currentDesiredOmega);
            // Return not done
            return false;
        } else {
            log("only turn");
            // Fix target
            currentDesiredVelocity = 0;
            // Calculate errors
            double errorTheta = calculateLastError();
            // Calculate desired angular velocity
            if (General.deadband(errorTheta, ANGLE_TOLERANCE) == 0) {
                currentDesiredOmega = 0;
            } else {
                currentDesiredOmega = errorTheta * K_THETA - getCurrentPoint().getCurvature() * K_OMEGA;
            }
            // Send drive command
            drive.driveVector(currentDesiredVelocity, currentDesiredOmega);
            // Return result
            return General.deadband(errorTheta, ANGLE_TOLERANCE) == 0;
        }
    }

    public Point getCurrentPoint() {
        return drive.getOdometry().toPoint();
    }

    public double[] calculateErrors() {
        // Theta calculation
        double errorTheta = Math.toRadians(General.compassify(Math.toDegrees(curvature(getCurrentPoint(), points.get(index)) - Math.toRadians(getCurrentPoint().getAngle()))));
        // Error calculation
        double currentDistanceError = distance(getCurrentPoint(), points.get(index));
        double previousDistanceError = distance(getCurrentPoint(), points.get(index - 1));
        // Return tuple
        return new double[]{currentDistanceError, previousDistanceError, errorTheta};
    }

    public double calculateLastError() {
        // Get the errors
        Point lastPose = points.get(points.size() - 1);
        // Return tuple
        return Math.toRadians(General.compassify(lastPose.getAngle() - getCurrentPoint().getAngle()));
    }

    private double[] deltas(Point first, Point last) {
        // Calculate errors
        double errorX = last.getX() - first.getX();
        double errorY = last.getY() - first.getY();
        // Return array
        return new double[]{errorX, errorY};
    }

    private double curvature(Point first, Point last) {
        // Calculate deltas
        double[] deltas = deltas(first, last);
        // Calculate angle
        return Math.atan2(deltas[1], deltas[0]);
    }

    private double distance(Point first, Point last) {
        // Assign values
        double[] errors = deltas(first, last);
        // Calculate and return
        return Math.sqrt(errors[0] * errors[0] + errors[1] * errors[1]);
    }

    @Override
    protected void log(String string) {
        if (LOGS_ENABLED)
            super.log(string);
    }

    private Trajectory.State pointToState(Point point) {
        Trajectory.State state = new Trajectory.State();
        state.poseMeters = new Pose2d(point.getX(), point.getY(), Rotation2d.fromDegrees(point.getAngle()));
        state.curvatureRadPerMeter = point.getCurvature();
        return state;
    }

    private Point stateToPoint(Trajectory.State state) {
        return new Point(state.poseMeters.getTranslation().getX(), state.poseMeters.getTranslation().getY(), state.poseMeters.getRotation().getDegrees(), state.curvatureRadPerMeter);
    }

    private void trajectoryToPoints(Trajectory trajectory) {
        points = new ArrayList<>();
        for (Trajectory.State state : trajectory.getStates()) {
            points.add(stateToPoint(state));
        }
    }
}
