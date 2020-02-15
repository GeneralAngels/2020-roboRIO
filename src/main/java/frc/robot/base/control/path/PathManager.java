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

    private static final double TOLERANCE = 0.05;
    private static final double TIME_DELTA = 0.02;
    private static final double MINIMUM_VELOCITY = 0.5;
    private static final double DEGREE_TOLERANCE = 3;

    private static final int FUTURE_CURVATURE_STATES = 2; // How many states to skip when looking for future curvatures (for linear acceleration)

    private DifferentialDrive drive;

    private Pose2d target;
    private Trajectory trajectory;

    // Trajectory progress
    private int index = 0;

    // Odometry
    private double theta, omega, x, y;

    // Errors
    private double[] lastErrors = new double[3];
    private double[] currentErrors = new double[3];

    private double maxVelocity = 1.5;
    private double maxAcceleration = 1.5;

    public double Kv = 3;
    public double Ktheta = 1.3 * maxVelocity; // Note that low values (for example 0.33) means it should get to the setpoint in ~3 seconds! // 1.6*MAX_V
    public double Kcurv = 1 * maxVelocity; //0.45*MAX_V
    public double Komega = 0;
    public boolean checkMaximumValues = true;
    public boolean finalDirectionSwitch = true;
    public double desiredOmega;
    public double currentDesiredVelocity;
    public double previousDesiredVelocity = 0;

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
                    createPath(new Pose2d(x, y, Rotation2d.fromDegrees(theta)));
                    return new Tuple<>(true, "Trajectory created");
                } else {
                    return new Tuple<>(false, "Wrong number of parameters");
                }
            }
        });

        command("follow", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                log("Index " + index + " len " + trajectory.getStates().size());
                if (index > trajectory.getStates().size()) {
                    return new Tuple<>(true, "Done");
                }
                follow();
                return new Tuple<>(false, "Not done");
            }
        });
    }

    public void follow() {
        // Follow trajectory
        // Setup previous values
        previousDesiredVelocity = currentDesiredVelocity;
        // Setup odometry value
        Odometry odometry = drive.getOdometry();
        theta = odometry.getTheta();
        omega = odometry.getOmega();
        x = odometry.getX();
        y = odometry.getY();
        // Setup states
        Trajectory.State start = trajectory.getStates().get(0);
        Trajectory.State current = trajectory.getStates().get(index);
        Trajectory.State end = trajectory.getStates().get(trajectory.getStates().size() - 1);
        //Setup magic
        if (checkMaximumValues) {

            checkMaximumValues = false;
        }
        // Calculate curvature
        double curvature = current.curvatureRadPerMeter;
        if (index < trajectory.getStates().size() - FUTURE_CURVATURE_STATES) // Check future curvature
            curvature = trajectory.getStates().get(index + FUTURE_CURVATURE_STATES).curvatureRadPerMeter; // Look for future curvature
        // Absolute the values
        curvature = Math.abs(curvature);
        // Calculate new errors
        double[] errors = calculateErrors(current);
        // Make sure we are not done yet
        if (index < trajectory.getStates().size()) {
            // Calculate average curvature
            double acceleration = maxAcceleration - (movingAverageCurvature() / 2.0); // 2.0 is an arbitrary value
            // Calculate velocity (maxVelocity - curvature * kCurv)(V = V0 + a*t)
            currentDesiredVelocity = Math.min(maxVelocity - (curvature * Kcurv), previousDesiredVelocity + (acceleration * TIME_DELTA));
            // Check range
            if ((currentDesiredVelocity >= 0 && currentDesiredVelocity < MINIMUM_VELOCITY) || (maxVelocity < (curvature * Kcurv))) // To make sure velocity isn't too low
                currentDesiredVelocity = MINIMUM_VELOCITY;
            // Calculate omega (PD control)
            desiredOmega = errors[2] * Ktheta - omega * Komega;
            if (errors[0] < errors[1])
                ++index;
        } else {
            // Reverse
            toReverse(getPose(), trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters);
            // Calculate desired sh*t
            currentDesiredVelocity = errors[0] * Kv;
            desiredOmega = (current.poseMeters.getRotation().getRadians() - Math.toRadians(Gyroscope.getAngle())) * Ktheta - omega * Komega;
            // Check if done
            if (!isDone(getPose(), trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters)) {
                if (errors[0] > TOLERANCE) {
                    desiredOmega *= 1.3;
                    log("close");
                } else {
                    currentDesiredVelocity = 0;
                    desiredOmega *= 1.8;
                    log("almost");
                }
                set("last point: ", "true");
            } else {
                currentDesiredVelocity = 0;
                desiredOmega = 0;
                log("done");
            }
        }
        if (index < trajectory.getStates().size() - 1 && errors[0] < errors[1]) {
            ++index;
        }
        drive.driveVector(currentDesiredVelocity, desiredOmega);
    }

    public double[] calculateErrors(Trajectory.State currentGoal) {
        // Previous error
        this.lastErrors = currentErrors;
        // Assign values
        double errorX = currentGoal.poseMeters.getTranslation().getX() - x;
        double errorY = currentGoal.poseMeters.getTranslation().getY() - y;
        // Theta calculation
        double errorTheta = (Math.atan2(errorY, errorX) - Math.toRadians(theta)) % (2 * Math.PI);
        // Return tuple
        return currentErrors = new double[]{Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)), lastErrors[1], errorTheta};
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    }

    public boolean isDone(Pose2d current, Pose2d end) {
        // Assign values
        double errorX = current.getTranslation().getX() - end.getTranslation().getX();
        double errorY = current.getTranslation().getY() - end.getTranslation().getY();
        // Calculate errors
        double errorTheta = Math.abs(current.getRotation().getDegrees() - end.getRotation().getDegrees());
        double errorPosition = errorX * Math.cos(Math.toRadians(theta)) + errorY * Math.sin(Math.toRadians(theta));
        // Check against tolerances
        return Math.abs(errorPosition) < TOLERANCE && errorTheta < DEGREE_TOLERANCE;
    }

    public void toReverse(Pose2d current, Pose2d end) {
        // Assign values
        double errorX = current.getTranslation().getX() - end.getTranslation().getX();
        double errorY = current.getTranslation().getY() - end.getTranslation().getY();
        // Calculate errors
        double errorTheta = Math.abs(current.getRotation().getDegrees() - end.getRotation().getDegrees());
        double errorPosition = errorX * Math.cos(Math.toRadians(theta)) + errorY * Math.sin(Math.toRadians(theta));
        // Black magic
        if ((errorPosition > 0 && Kv > 0) || (errorPosition < 0 && Kv < 0)) {
            if (finalDirectionSwitch) {
                Kv *= -1;
                finalDirectionSwitch = false;
            }
        } else
            finalDirectionSwitch = true;
    }

    public void createPath(Pose2d target) {
        // Reset index
        index = 1;
        // Configure trajectory
        TrajectoryConfig config = new TrajectoryConfig(2, 1);
        config.setEndVelocity(0);
        // Poses
        Pose2d start = getPose();
        Pose2d end = new Pose2d(start.getTranslation().getX() + target.getTranslation().getX(), start.getTranslation().getY() + target.getTranslation().getY(), Rotation2d.fromDegrees(start.getRotation().getDegrees() + target.getRotation().getDegrees()));
        // Calculate deltas
        double xDelta = end.getTranslation().getX() - start.getTranslation().getX();
        double yDelta = end.getTranslation().getY() - start.getTranslation().getY();
        // Calculate curvature
        double trajectoryCurvature = Math.atan2(yDelta, xDelta);
        // Calculate the maximum acceleration and velocity
        maxVelocity = Math.min(0.75 / Math.abs(trajectoryCurvature), 1.5);
        maxAcceleration = Math.min(0.5 / Math.abs(trajectoryCurvature), 1.5);
        // Generate trajectory
        trajectory = TrajectoryGenerator.generateTrajectory(start, new ArrayList<>(), end, config);
    }

    public double movingAverageCurvature() {
        double average = 0;
        for (int i = trajectory.getStates().size() - 1; i >= 0; i--) {
            average = (average + Math.abs(trajectory.getStates().get(i).curvatureRadPerMeter) / 2);
        }
        return average;
    }

    public double[] derivePolynomial(double[] coefficients) {
        double[] result = new double[coefficients.length];
        result[0] = 0;
        for (int i = 1; i < coefficients.length; i++) {
            result[i] = coefficients[i - 1] * i;
        }
        return result;
    }
}
