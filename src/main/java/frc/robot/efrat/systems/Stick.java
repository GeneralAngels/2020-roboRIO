package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.PinMan;
import org.json.JSONObject;

public class Stick extends Subsystem {
    private static final String LOCATION = "location";
    private static final String TARGET = "target_location";
    private static final double STICK_LENGTH_METERS = 1.22;
    private static final double RADIUS = 0.05;
    private static Stick latest;
    private double location = 0;
    private double targetLocation = 0;
    private WPI_TalonSRX motor;
    private DigitalInput startReset, endReset;
    private Encoder encoder;

    public Stick() {
        latest = this;
        PinMan pinManager = new PinMan();
        motor = new WPI_TalonSRX(15);
        encoder = new Encoder(8, 9);
//        frontReset = new DigitalInput(2);
//        endReset = new DigitalInput(3);
    }

    public static Stick getInstance() {
        return latest;
    }

    private void calculateLocation() {
        // TODO Calculate "location"
    }

    public void setTargetLocation(double targetLocation) {
        this.targetLocation = targetLocation;
    }

    public void loop() {
        if (startReset != null) {
            if (!startReset.get()) {
                // Pressed
                location = 0;
            }
        }
        if (startReset != null) {
            if (!endReset.get()) {
                // Pressed
                location = STICK_LENGTH_METERS;
            }
        }
        calculateLocation();
        // Do PID Stuff
    }

    public void set(double speed) {
        speed /= 3;
        motor.set(speed);
    }

    @Override
    public JSONObject toJSON() {
        JSONObject object = super.toJSON();
        object.put(LOCATION, location);
        object.put(TARGET, targetLocation);
        return object;
    }
}