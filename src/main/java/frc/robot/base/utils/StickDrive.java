package frc.robot.base.utils;

public class StickDrive {

    public static double right(double y, double x){
        return (x + y);
    }

    public static double left(double y, double x){
        return (x - y);
    }
}
