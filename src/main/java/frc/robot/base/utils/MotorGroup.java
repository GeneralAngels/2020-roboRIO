package frc.robot.base.utils;

import com.ga2230.shleam.advanced.frc.FRCModule;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

import java.util.ArrayList;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class MotorGroup<Type extends SpeedController> extends FRCModule {

    public static final int FORWARD = 1, BACKWARD = -1, BRAKE = 0;

    private Encoder encoder;
    private ArrayList<Type> drives = new ArrayList<>();
    private int direction = FORWARD;

    public MotorGroup(String id) {
        super(id);
    }

    public MotorGroup(String id, Type[] drives) {
        super(id);
        addMotors(drives);
    }

    public MotorGroup(String id, Encoder encoder) {
        super(id);
        setEncoder(encoder);
    }

    public MotorGroup(String id, Encoder encoder, Type[] drives) {
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

    public boolean hasEncoder() {
        return encoder != null;
    }

    public void resetEncoder() {
        if (hasEncoder())
            encoder.reset();
    }

    public int getDirection() {
        return direction;
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }

    public Type[] addMotors(Type[] drives) {
        for (Type drive : drives) addMotor(drive);
        return drives;
    }

    public Type addMotor(Type drive) {
        drives.add(drive);
        return drive;
    }

    public void removeMotor(Type drive) {
        drives.remove(drive);
    }

    public void applyPower(double value) {
        set("speed", String.valueOf(value));
        set("encoder", String.valueOf(encoder != null ? encoder.get() : 0));
        for (Type drive : drives) drive.set(value * direction);
    }
}

