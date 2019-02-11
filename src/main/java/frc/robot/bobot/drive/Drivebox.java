package frc.robot.bobot.drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.bobot.Subsystem;
import org.json.JSONObject;

import java.util.ArrayList;

public class Drivebox<T extends SpeedController> extends Subsystem {

    public static final String SPEED = "speed";
    public static final String ENCODER = "encoder";
    public static final int DIRECTION_FORWARD = 1, DIRECTION_BACKWARD = -1, DIRECTION_BRAKE = 0;

    protected Encoder encoder;
    protected ArrayList<T> drives = new ArrayList<>();
    protected int direction = DIRECTION_FORWARD;
    protected double speed = 0;

    public Drivebox() {

    }

    public Drivebox(T[] drives) {
        add(drives);
    }

    public Drivebox(Encoder encoder) {
        setEncoder(encoder);
    }

    public Drivebox(Encoder encoder, T[] drives) {
        add(drives);
        setEncoder(encoder);
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public Drivebox setEncoder(Encoder encoder) {
        this.encoder = encoder;
        return this;
    }

    public int getDirection() {
        return direction;
    }

    public Drivebox setDirection(int direction) {
        this.direction = direction;
        return this;
    }

    public Drivebox add(T[] drives) {
        for (T drive : drives) add(drive);
        return this;
    }

    public Drivebox add(T drive) {
        drives.add(drive);
        return this;
    }

    public Drivebox remove(T drive) {
        drives.remove(drive);
        return this;
    }

    public Drivebox set(double value) {
        this.speed = value;
        for (T drive : drives) drive.set(value * direction);
        return this;
    }

    @Override
    public JSONObject toJSON() {
        JSONObject jsonObject = new JSONObject();
        try {
            jsonObject.put(SPEED, speed);
            if (encoder != null) {
                jsonObject.put(ENCODER, encoder.get());
            }
        } catch (Exception ignored) {

        }
        return jsonObject;
    }
}

