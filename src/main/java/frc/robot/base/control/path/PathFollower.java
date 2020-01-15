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
        });
    }

    public Trajectory createPath() {
        //we don't need the abcdk. the library does this itself

        Pose2d startPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d endPoint = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
//        interiorWaypoints.add(new Translation2d(0.5, 1.0));
//        interiorWaypoints.add(new Translation2d(1.0, 0.5));

        TrajectoryConfig config = new TrajectoryConfig(4, 2);
        //config.setReversed(true);
        Trajectory res = TrajectoryGenerator.generateTrajectory(startPoint, interiorWaypoints, endPoint, config);
        log("trajectory:" + res.toString());
        res.getStates();
        return res;

    }

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
