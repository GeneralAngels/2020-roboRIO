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

public class PathManager extends frc.robot.base.Module {

    private Trajectory trajectory;
    private double[][] Ainverse = new double[][]{{2,1,-2,1},{-3,-2,3,-1},{0,1,0,0},{1,0,0,0}};
    private double[][] A = new double[][]{{0,0,0,1},{0,0,1,0},{1,1,1,1},{3,2,1,0}};

    public PathManager(DifferentialDrive drive) {
        super("pathman");
        command("create_with_abcd", new Command() {
            @Override
            public String execute(String s) throws Exception {
                String[] split = s.split(" ");
                if (split.length == 4) {
                    drive.setTrajectory(createTrajectory(Double.parseDouble(split[0]), Double.parseDouble(split[1]), Double.parseDouble(split[2]), Double.parseDouble(split[3])));
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
        double[][] points = new double[amount][3];

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

    public Trajectory createTrajectory(double a, double b, double c, double d) {
        return null;
    }

    public Trajectory createPath() {
        //we don't need the abcdk. the library does this itself

        Pose2d startPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d endPoint = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
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

    private double[] dot(double[][] A, double[] y){ //Solving for Ax = y (A=Matrix, a=vector, y = vector)
        double[] res = new double[y.length];
        for (int j = 0; j < y.length; j++) {
            double rowProduct = 0;
            for (int i = 0; i < A[0].length; i++) {
                rowProduct += A[j][i]*y[i];
            }
            res[j] = rowProduct;
        }
        return res;
    }

    /**
     * @param start start x
     * @param dS direction at the start
     * @param end end X
     * @param dE direction at the end
     * @return the coefficients
     */
    public double[] createSpline(double start, double dS, double end, double dE){
        double[] y = new double[]{start,dS,end,dE};
        return dot(Ainverse,y);
    }

    public double[] derivePolynom(double[] coefs){
        double[] res = new double[coefs.length];
        res[0] = 0;
        for (int i = 1; i < coefs.length; i++) {
            res[i] = coefs[i-1]*i;
        }
        return res;
    }



}
