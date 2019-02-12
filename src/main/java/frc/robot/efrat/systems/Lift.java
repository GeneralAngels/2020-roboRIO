package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.control.PID;
import frc.robot.bobot.utils.PinManager;

public class Lift extends Subsystem {

    public static final double MAX_ANGLE_RADIANS = 1;
    public static final double[] ROCKET_ANGLES_RADIANS = {0, 0.5, 0.9};
    private double currentAngle = 0;
    private int targetLevel = 0;
    private double targetAngleRadians = 0;
    private double sensorOffset;
    public AnalogInput potentiometer;
    PID motor1PID, motor2PID;
    // todo remove
    private DigitalInput upReset, downReset;
    public WPI_TalonSRX motor1, motor2;

    public Lift() {
        PinManager pinManager = new PinManager();
        motor1 = new WPI_TalonSRX(15);
        motor2 = new WPI_TalonSRX(16);
        potentiometer = new AnalogInput(2);
        downReset = new DigitalInput(0);
        upReset = new DigitalInput(1);
        motor1PID = new PID();
        motor1PID.setPIDF(0,0,0,0);
        motor2PID = new PID();
        motor2PID.setPIDF(0,0,0,0);
    }

    public void setTargetLevel(int targetLevel) {
        if (targetLevel < ROCKET_ANGLES_RADIANS.length) {
            this.targetLevel = targetLevel;
            targetAngleRadians = ROCKET_ANGLES_RADIANS[targetLevel];
        }
    }

    private void calculateAngle() {
        currentAngle = (potentiometer.getVoltage() - sensorOffset) / (256 * MAX_ANGLE_RADIANS);
    }

    public void loop() {
        if (!downReset.get()) {
            // Down Pressed
            sensorOffset = potentiometer.getAverageVoltage();
        }
        calculateAngle();

        double motor1Velocity = motor1PID.pidVelocity(currentAngle, targetAngleRadians);
        double motor2Velocity = motor2PID.pidVelocity(currentAngle, targetAngleRadians);

    }

    public void set(double speed) {
        // why /6?
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
