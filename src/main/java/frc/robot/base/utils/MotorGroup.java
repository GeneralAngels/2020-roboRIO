package frc.robot.base.utils;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.base.Module;
import org.json.JSONObject;

import java.util.ArrayList;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class MotorGroup<Type extends SpeedController> extends Module {

    public static final int FORWARD = 1, BACKWARD = -1, BRAKE = 0;

    private Encoder encoder;
    private ArrayList<Type> drives = new ArrayList<>();
    private int direction = FORWARD;
    private double speed = 0;

    public MotorGroup() {

    }

    public MotorGroup(Type... drives) {
        addMotors(drives);
    }

    public MotorGroup(Encoder encoder) {
        setEncoder(encoder);
    }

    public MotorGroup(Encoder encoder, Type... drives) {
        setEncoder(encoder);
        addMotors(drives);
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public void setEncoder(Encoder encoder) {
        this.encoder = encoder;
    }

    public int getDirection() {
        return direction;
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }

    public void addMotors(Type... drives) {
        for (Type drive : drives) addMotor(drive);
    }

    public void addMotor(Type drive) {
        drives.add(drive);
    }

    public void removeMotor(Type drive) {
        drives.remove(drive);
    }

    public void applyPower(double value) {
        this.speed = value;
        for (Type drive : drives) drive.set(value * direction);
    }

    @Override
    public JSONObject pullJSON() {
        log("power", speed);
        log("encoder", encoder != null ? encoder.get() : 0);
        return super.pullJSON();
    }
}

