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
import org.opencv.core.Mat;

import java.util.ArrayList;

public class PathManager extends FRCModule {

    private static final double TIME_DELTA = 0.02;

    private static final double DISTANCE_TOLERANCE = 0.05;
    private static final double RADIAN_TOLERANCE = 3 * Math.PI / 180;
    private static final double DEGREE_TOLERANCE = 5;

    private static final boolean LOGS_ENABLED = true;

    private static final int FUTURE_CURVATURE_STATES = 2; // How many states to skip when looking for future curvatures (for linear acceleration)

    private DifferentialDrive drive;

    private ArrayList<Point> points;

    private Trajectory trajectory;

    // Trajectory progress
    private int index = 0;

    private double theta, omega, x, y;
    private double minVelocity, maxVelocity, maxAcceleration;

    public double kTheta, kCurvature, kOmega, kVelocity;
    public double currentDesiredOmega, currentDesiredVelocity, previousDesiredVelocity;

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
                if (trajectory != null) {
                    updateProgress();
                    if (index >= trajectory.getStates().size()) {
                        return Result.finished("Done");
                    }
                    followTrajectory(parameter.equals("reverse"));
                    return Result.notFinished("Not done");
                } else {
                    return Result.notFinished("No trajectory");
                }
            }
        });
    }

    private void updateProgress() {
        set("index", String.valueOf(index));
        set("length", String.valueOf(trajectory.getStates().size()));
    }

    private void updateOdometry() {
        Odometry odometry = drive.updateOdometry();
//        theta = General.compassify(odometry.getTheta());
        theta = odometry.getTheta();
        omega = odometry.getOmega();
        x = odometry.getX();
        y = odometry.getY();
    }

    public void createTrajectory(Point target, boolean reversed) {
        // Reset array
        points = new ArrayList<>();
        // Reset index
        index = 1;
        // Reset things
        kOmega = 0.05;
        kVelocity = 2;
        maxVelocity = 2;
        maxAcceleration = 2;
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
        // Calculate curvature
        double trajectoryCurvature = curvature(end, start);
        // Calculate the maximum acceleration and velocity
//        maxVelocity = Math.min(4 / Math.abs(trajectoryCurvature), 1.5);
        double angleDelta = Math.abs(Math.sin(Math.toRadians(end.getAngle() - start.getAngle())));
        double yDelta = Math.abs(end.getY() - start.getY());
        // Calculate max acceleration
        if (yDelta > 0.4)
            maxAcceleration = Math.min((3 / (Math.abs(trajectoryCurvature)) + (angleDelta * yDelta)), maxAcceleration);
        else
            maxAcceleration = Math.min(3.5 / Math.abs(trajectoryCurvature), maxAcceleration);
        // Calculate minimum velocity
        minVelocity = 0.5;
        // Reset some more things
        kTheta = 3.5;
        kCurvature = 4;
        // Multiply for reversed
        if (reversed) {
            kCurvature *= -1;
            maxAcceleration *= -1;
        }
    }

    public void followTrajectory(boolean reversed) {
        // Follow trajectory
        updateProgress();
        // Check if finished
        if (index < points.size()) {
            // Setup previous values
            previousDesiredVelocity = currentDesiredVelocity;
            // Setup odometry value
            updateOdometry();
            // Setup states
            Point currentGoal = points.get(index);
            // Calculate curvature
            double curvature = currentGoal.getCurvature();
            if (index < points.size() - FUTURE_CURVATURE_STATES) // Check future curvature
                curvature = points.get(index + FUTURE_CURVATURE_STATES).getCurvature(); // Look for future curvature
            // Absolute the values
            curvature = Math.abs(curvature);
            // Calculate new errors
            double[] errors = calculateErrors();
            // Is last point yet?
            boolean isLast = !(index < points.size() - 1);
            // Calculate kTheta
            kTheta = Math.abs(kTheta);
            // Make sure we are not done yet
            if (!isLast) {
                // Calculate average curvature
                double acceleration = maxAcceleration - (movingAverageCurvature() / 2.0); // 2.0 is an arbitrary value
                // Sets
                set("maxV", String.valueOf(maxVelocity));
                set("maxA", String.valueOf(maxAcceleration));
                // Calculate velocity (maxVelocity - curvature * kCurv)(V = V0 + a*t)
                currentDesiredVelocity = Math.min(maxVelocity - (curvature * kCurvature), previousDesiredVelocity + (acceleration * TIME_DELTA));
                // Check range
                if ((currentDesiredVelocity >= 0 && currentDesiredVelocity < minVelocity) || (maxVelocity < (curvature * kCurvature))) // To make sure velocity isn't too low
                    currentDesiredVelocity = minVelocity;
                // Calculate omega (PD control)
                currentDesiredOmega = errors[2] * kTheta - omega * kOmega;
            } else {
                // Last point
                calculateVelocityCoefficient();
                // Calculate errors
                double[] lastErrors = calculateLastErrors();
                // Calculate desired sh*t
                currentDesiredVelocity = lastErrors[0] * kVelocity;
                currentDesiredOmega = lastErrors[2] * kTheta - omega * kOmega;
                // Check if done
                if (!(Math.abs(lastErrors[1]) < DISTANCE_TOLERANCE && Math.abs(lastErrors[2]) < RADIAN_TOLERANCE)) {
                    // Distance
                    if (Math.abs(lastErrors[1]) > DISTANCE_TOLERANCE) {
                        currentDesiredVelocity *= 1;
                    } else {
                        currentDesiredVelocity = 0;
                    }
                    if (Math.abs(lastErrors[2]) > RADIAN_TOLERANCE) {
                        currentDesiredOmega *= 3;
                    } else {
                        currentDesiredOmega = 0;
                    }
                } else {
                    currentDesiredVelocity = 0;
                    currentDesiredOmega = 0;
                }
                // Deadbands
                currentDesiredVelocity = General.deadband(currentDesiredVelocity, 0.1);
                currentDesiredOmega = General.deadband(currentDesiredOmega, 0.1);
                // Check if done (actually)
                if (currentDesiredVelocity == 0 && currentDesiredOmega == 0) {
                    this.index++;
                }
            }
            if (!isLast && errors[0] < errors[1]) {
                this.index++;
            }
            set("targetVelocity", String.valueOf(currentDesiredVelocity));
            set("targetOmega", String.valueOf(currentDesiredOmega));
            drive.driveVector(currentDesiredVelocity, currentDesiredOmega);
        } else {
            log("Finished");
        }
    }

    public Point getCurrentPoint() {
        return new Point(x, y, theta, omega);
    }

    public double[] calculateErrors() {
        // Theta calculation
        double errorTheta = (curvature(getCurrentPoint(), points.get(index)) - Math.toRadians(getCurrentPoint().getAngle())) % (2 * Math.PI);
        // Error calculation
        double currentDistanceError = absoluteDistance(getCurrentPoint(), points.get(index));
        double previousDistanceError = absoluteDistance(getCurrentPoint(), points.get(index - 1));
        // Return tuple
        return new double[]{currentDistanceError, previousDistanceError, errorTheta};
    }

    public double[] calculateLastErrors() {
        // Get the errors
        Point lastPose = points.get(points.size() - 1);
        // Calculate errors
        double relativeErrorPosition = relativeDistance(getCurrentPoint(), lastPose);
        double absoluteErrorPosition = absoluteDistance(getCurrentPoint(), lastPose);
        // Calculate angle error
        double errorTheta = Math.toRadians(points.get(points.size() - 1).getAngle() - getCurrentPoint().getAngle());
        // Return tuple
        return new double[]{absoluteErrorPosition, relativeErrorPosition, errorTheta};
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

    private double absoluteDistance(Point first, Point last) {
        // Assign values
        double[] errors = deltas(first, last);
        // Calculate and return
        return Math.sqrt(errors[0] * errors[0] + errors[1] * errors[1]);
    }

    private double relativeDistance(Point first, Point last) {
        // Assign values
        double[] errors = deltas(first, last);
        // Radian degrees of first (instead of theta)
        double angle = Math.toRadians(first.getAngle());
        // Return distance
        return errors[0] * Math.cos(angle) + errors[1] * Math.sin(angle);
    }

    private void calculateVelocityCoefficient() {
        // Calculate errors
        double errorDistance = relativeDistance(getCurrentPoint(), points.get(index));
        kVelocity = Math.abs(kVelocity);
        if (errorDistance < 0) {
            kVelocity *= -1;
        } else {
            kVelocity *= 1;
        }
    }

    private double movingAverageCurvature() {
        double average = 0;
        for (int i = points.size() - 1; i >= 0; i--) {
            average = (average + Math.abs(points.get(i).getCurvature())) / 2;
        }
        return average;
    }

    @Override
    protected void log(String string) {
        if (LOGS_ENABLED)
            super.log(string);
    }

    private double[] derivePolynomial(double[] coefficients) {
        double[] result = new double[coefficients.length];
        result[0] = 0;
        for (int i = 1; i < coefficients.length; i++) {
            result[i] = coefficients[i - 1] * i;
        }
        return result;
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
