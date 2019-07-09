package frc.robot.base.utils;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class StickDrive {

    public static double right(double y, double x){
        return (x + y);
    }

    public static double left(double y, double x){
        return (x - y);
    }
}
