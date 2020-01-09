package frc.robot.base.control.path;

import frc.robot.base.utils.MotorGroup;

import java.util.ArrayList;

public class PathFollower extends frc.robot.base.Module {
    public PathFollower() {
        super("follower");
        command("createpath", new Command() {
            @Override
            public String execute(String s) throws Exception {
                String[] split = s.split(" ");
                if (split.length == 5) {
                    createPath(Double.parseDouble(split[0]), Double.parseDouble(split[1]), Double.parseDouble(split[2]), Double.parseDouble(split[3]), Double.parseDouble(split[3]));
                }
                return "OK";
            }
        });
    }

    public void createPath(double a, double b, double c, double d, double k){

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
