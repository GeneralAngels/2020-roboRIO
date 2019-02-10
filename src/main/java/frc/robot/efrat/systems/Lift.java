package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.PinManager;

public class Lift extends Subsystem {

    public static final double MAX_ANGLE_RADIANS = 1;
    public static final double[] ROCKET_ANGLES_RADIANS = {0, 0.5, 0.9};
    private double currentAngle = 0;
    private int targetLevel = 0;
    private double targetAngleRadians = 0;
    private double sensorOffset;
    public AnalogInput potentiometer;
    // todo remove
    private DigitalInput upReset, downReset;
    private WPI_TalonSRX motor1, motor2;

    public Lift() {
        PinManager pinManager = new PinManager();
        motor1 = new WPI_TalonSRX(5);
        motor2 = new WPI_TalonSRX(6);
        potentiometer = new AnalogInput(2);
        downReset = new DigitalInput(0);
        upReset = new DigitalInput(1);
    }

    public void setTargetLevel(int targetLevel) {
        if (targetLevel < ROCKET_ANGLES_RADIANS.length) {
            this.targetLevel = targetLevel;
            targetAngleRadians = ROCKET_ANGLES_RADIANS[targetLevel];
        }
    }

    private void calculateAngle() {
        currentAngle = (potentiometer.getAverageVoltage() - sensorOffset) / (256 * MAX_ANGLE_RADIANS);
    }

    public void loop() {
        if (!downReset.get()) {
            // Down Pressed
            sensorOffset = potentiometer.getAverageVoltage();
        }
        calculateAngle();
        // Do PID Stuff
    }

    public void set(double speed) {
        speed/=6;
        motor1.set(speed);
        motor2.set(speed);
    }

    public void levelUp() {
        if (targetLevel < ROCKET_ANGLES_RADIANS.length - 1) {
            targetLevel++;
            targetAngleRadians = ROCKET_ANGLES_RADIANS[targetLevel];
        }
    }

    public void levelDown() {
        if (targetLevel > 0) {
            targetLevel--;
            targetAngleRadians = ROCKET_ANGLES_RADIANS[targetLevel];
        }
    }

}
