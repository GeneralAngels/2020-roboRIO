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

    public MotorGroup(String id) {
        super(id);
    }

    public MotorGroup(String id, Type... drives) {
        super(id);
        addMotors(drives);
    }

    public MotorGroup(String id, Encoder encoder) {
        super(id);
        setEncoder(encoder);
    }

    public MotorGroup(String id, Encoder encoder, Type... drives) {
        super(id);
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
        variables.put("power", String.valueOf(speed));
        variables.put("encoder", String.valueOf(encoder != null ? encoder.get() : 0));
        return super.pullJSON();
    }
}

