package frc.robot.base.control.path;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.drive.Gyroscope;
import frc.robot.base.drive.Odometry;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;

public class PathManager extends frc.robot.base.Module {

    private static final double TIME_DELTA = 0.02;

    private static final double MINIMUM_VELOCITY = 0.5;

    private static final double DISTANCE_TOLERANCE = 0.05;
    private static final double DEGREE_TOLERANCE = 1;

    private static final int FUTURE_CURVATURE_STATES = 2; // How many states to skip when looking for future curvatures (for linear acceleration)

    private DifferentialDrive drive;

    private Pose2d target;
    private Trajectory trajectory;

    // Trajectory progress
    private int index = 0;

    private double theta, omega, x, y;

    private double maxVelocity = 1.5;
    private double maxAcceleration = 1.5;

    public double kTheta = 1.3 * maxVelocity; // Note that low values (for example 0.33) means it should get to the setpoint in ~3 seconds! // 1.6*MAX_V
    public double kCurvature = 1 * maxVelocity;
    public double kOmega = 0;

    public double currentDesiredOmega;
    public double currentDesiredVelocity;
    public double previousDesiredVelocity;

    public double kV = 1;

    public PathManager(DifferentialDrive drive) {
        super("path");
        this.drive = drive;

        // Command registration for autonomous

        command("fetch", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (trajectory != null) {
                    JSONArray array = new JSONArray();
                    for (Trajectory.State state : trajectory.getStates()) {
                        JSONObject object = new JSONObject();
                        object.put("x", state.poseMeters.getTranslation().getX());
                        object.put("y", state.poseMeters.getTranslation().getY());
                        object.put("angle", state.poseMeters.getRotation().getDegrees());
                        array.put(object);
                    }
                    return new Tuple<>(true, array.toString());
                }
                return new Tuple<>(true, "[]");
            }
        });

        command("create", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                // Parse string into three parameters (x, y, theta)
                String[] split = s.split(" ");
                if (split.length == 3) {
                    double x = Double.parseDouble(split[0]);
                    double y = Double.parseDouble(split[1]);
                    double theta = Double.parseDouble(split[2]);
                    createTrajectory(new Pose2d(x, y, Rotation2d.fromDegrees(theta)));
                    return new Tuple<>(true, "Trajectory created");
                } else {
                    return new Tuple<>(false, "Wrong number of parameters");
                }
            }
        });

        command("follow", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (trajectory != null) {
                    updateProgress();
                    if (index >= trajectory.getStates().size()) {
                        return new Tuple<>(true, "Done");
                    }
                    followTrajectory();
                    return new Tuple<>(false, "Not done");
                } else {
                    return new Tuple<>(false, "No trajectory");
                }
            }
        });
    }

    private void updateProgress() {
        set("index", String.valueOf(index));
        set("length", String.valueOf(trajectory.getStates().size()));
        set("progress", get("index") + "/" + get("length"));
    }

    private void updateOdometry() {
        Odometry odometry = drive.updateOdometry();
        theta = odometry.getTheta();
        omega = odometry.getOmega();
        x = odometry.getX();
        y = odometry.getY();
    }

    public void createTrajectory(Pose2d target) {
        // Reset index
        index = 1;
        // Update odometry
        updateOdometry();
        // Configure trajectory
        TrajectoryConfig config = new TrajectoryConfig(2, 1);
        config.setEndVelocity(0);
        // Poses
        Pose2d start = getPose();
        Pose2d end = new Pose2d(start.getTranslation().getX() + target.getTranslation().getX(), start.getTranslation().getY() + target.getTranslation().getY(), Rotation2d.fromDegrees(start.getRotation().getDegrees() + target.getRotation().getDegrees()));
        // Calculate curvature
        double trajectoryCurvature = curvature(end, start);
        // Calculate the maximum acceleration and velocity
        maxVelocity = Math.min(0.75 / Math.abs(trajectoryCurvature), 1.5);
        maxAcceleration = Math.min(0.5 / Math.abs(trajectoryCurvature), 1.5);
        // Generate trajectory
        trajectory = TrajectoryGenerator.generateTrajectory(start, new ArrayList<>(), end, config);
    }

    public void followTrajectory() {
        // Follow trajectory
        updateProgress();
        // Check if finished
        if (index < trajectory.getStates().size()) {
            // Setup previous values
            previousDesiredVelocity = currentDesiredVelocity;
            // Setup odometry value
            updateOdometry();
            // Setup states
            Trajectory.State current = trajectory.getStates().get(index);
            // Calculate curvature
            double curvature = current.curvatureRadPerMeter;
            if (index < trajectory.getStates().size() - FUTURE_CURVATURE_STATES) // Check future curvature
                curvature = trajectory.getStates().get(index + FUTURE_CURVATURE_STATES).curvatureRadPerMeter; // Look for future curvature
            // Absolute the values
            curvature = Math.abs(curvature);
            // Calculate new errors
            double[] errors = calculateErrors();
            // Is last point yet?
            boolean isLast = !(index < trajectory.getStates().size() - 1);
            // Make sure we are not done yet
            if (!isLast) {
                // Calculate average curvature
                double acceleration = maxAcceleration - (movingAverageCurvature() / 2.0); // 2.0 is an arbitrary value
                // Calculate velocity (maxVelocity - curvature * kCurv)(V = V0 + a*t)
                currentDesiredVelocity = Math.min(maxVelocity - (curvature * kCurvature), previousDesiredVelocity + (acceleration * TIME_DELTA));
                // Check range
                if ((currentDesiredVelocity >= 0 && currentDesiredVelocity < MINIMUM_VELOCITY) || (maxVelocity < (curvature * kCurvature))) // To make sure velocity isn't too low
                    currentDesiredVelocity = MINIMUM_VELOCITY;
                // Calculate omega (PD control)
                currentDesiredOmega = errors[2] * kTheta - omega * kOmega;
            } else {
                // Last point
                calculateVelocityCoefficient();
                // Calculate desired sh*t
                currentDesiredVelocity = errors[0] * kV;
                currentDesiredOmega = (current.poseMeters.getRotation().getRadians() - Math.toRadians(Gyroscope.getAngle())) * kTheta - omega * kOmega;
                // Check if done
                if (!(errors[0] < DISTANCE_TOLERANCE && errors[2] < DEGREE_TOLERANCE)) {
                    // Distance
                    if (errors[0] > DISTANCE_TOLERANCE) {
                        currentDesiredVelocity *= 1.8;
                    } else {
                        currentDesiredVelocity = 0;
                    }
                    if (errors[2] > DEGREE_TOLERANCE) {
                        currentDesiredOmega *= 1.6;
                    } else {
                        currentDesiredOmega = 0;
                    }
                } else {
                    currentDesiredVelocity = 0;
                    currentDesiredOmega = 0;
                    this.index++;
                }
            }
            if (!isLast && errors[0] < errors[1]) {
                this.index++;
            }
            drive.driveVector(currentDesiredVelocity, currentDesiredOmega);
        }
    }

    public double[] calculateErrors() {
        // Theta calculation
        double errorTheta = (curvature(getPose(), trajectory.getStates().get(index).poseMeters) - Math.toRadians(theta)) % (2 * Math.PI);
        // Error calculation
        double currentDistanceError = Math.abs(distance(getPose(), trajectory.getStates().get(index).poseMeters));
        double previousDistanceError = Math.abs(distance(getPose(), trajectory.getStates().get(index - 1).poseMeters));
        // Return tuple
        return new double[]{currentDistanceError, previousDistanceError, errorTheta};
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    }

    private double[] deltas(Pose2d first, Pose2d last) {
        // Calculate errors
        double errorX = last.getTranslation().getX() - first.getTranslation().getX();
        double errorY = last.getTranslation().getY() - first.getTranslation().getY();
        // Return array
        return new double[]{errorX, errorY};
    }

    private double curvature(Pose2d first, Pose2d last) {
        // Calculate deltas
        double[] deltas = deltas(first, last);
        // Calculate angle
        return Math.atan2(deltas[1], deltas[0]);
    }

    private double distance(Pose2d first, Pose2d last) {
        // Assign values
        double[] errors = deltas(first, last);
        // Radian degrees of first (instead of theta)
        double angle = first.getRotation().getRadians();
        // Return distance
        return errors[0] * Math.cos(angle) + errors[1] * Math.sin(angle);
    }

    private void calculateVelocityCoefficient() {
        // Calculate errors
        double errorDistance = distance(getPose(), trajectory.getStates().get(index).poseMeters);
        if ((kV < 0) != (errorDistance < 0)) {
            kV *= -1;
        }
    }

    private double movingAverageCurvature() {
        double average = 0;
        for (int i = trajectory.getStates().size() - 1; i >= 0; i--) {
            average = (average + Math.abs(trajectory.getStates().get(i).curvatureRadPerMeter)) / 2;
        }
        return average;
    }

    private double[] derivePolynomial(double[] coefficients) {
        double[] result = new double[coefficients.length];
        result[0] = 0;
        for (int i = 1; i < coefficients.length; i++) {
            result[i] = coefficients[i - 1] * i;
        }
        return result;
    }
}
