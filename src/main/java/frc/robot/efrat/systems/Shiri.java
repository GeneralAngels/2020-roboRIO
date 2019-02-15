package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.control.PID;
import frc.robot.bobot.utils.PinMan;

public class Shiri extends Subsystem {

    private static Shiri latest;
    private static final double DISTANCE = 0.66;
    private static final double LIMIT_V = 0.5;
    private static final double RADIUS = 0.0191; //TODO: check value
    private static final double TICKS_PER_REVOLUTIONS = 1024; //TODO: check value
    private static final double ENC_TO_METERS = (2*3.14*RADIUS) / (4*TICKS_PER_REVOLUTIONS);
    private DigitalInput frontReset, backReset, grab1, grab2;
    private DoubleSolenoid hatch;
    private WPI_TalonSRX slideMotor;
    private PID xPID;
    private double xPrev = 0;

    public Shiri() {
        latest = this;
        hatch = new DoubleSolenoid(0, 1);
        slideMotor = new WPI_TalonSRX(14);
//        slideMotor.setInverted(true);
        grab1 = new DigitalInput(PinMan.getNavDIO(1));
        grab2 = new DigitalInput(PinMan.getNavDIO(2));
        backReset = new DigitalInput(PinMan.getNavDIO(3));
        frontReset = new DigitalInput(PinMan.getNavDIO(4));
        xPID = new PID();
        xPID.setPIDF(0.9, 0.8, 0, 0);
    }

    public static Shiri getInstance() {
        return latest;
    }

    public boolean isAtFront() {
        return frontReset.get();
    }

    public boolean isAtBack() {
        log("Shiri Position-Reset");
        return backReset.get();
    }

    public void moveToBack() {

    }

    public boolean isHatchLoaded() {
        return grab1 != null && grab2 != null && !grab1.get() && grab2.get();
    }

    public boolean isHatchLocked() {
        return hatch != null && hatch.get() == DoubleSolenoid.Value.kForward;
    }

    public void open() {
        if (hatch != null) hatch.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        if (hatch != null) hatch.set(DoubleSolenoid.Value.kReverse);
    }

    public void setPower(double speed){
        if (speed > 0) {
            if (!frontReset.get())
                slideMotor.set(speed);
            else
                slideMotor.set(0);
        } else if (speed < 0) {
            if (!backReset.get())
                slideMotor.set(speed);
            else
                slideMotor.set(0);
        } else {
            slideMotor.set(speed);
        }
    }

    public void set(double x) { //changed method
        double slideMotor_output = controlX(x);
        log("pid shiri:" + slideMotor_output);
        slideMotor.set(-slideMotor_output);
    }

    public int sign(double a){
        if (a>0)
            return 1;
        else if (a<0)  return -1;
        return 0;
    }
    public void print(){
        log("meters: "+(0.54-((slideMotor.getSelectedSensorPosition()/10) * ENC_TO_METERS)));
        log("encoder: "+(slideMotor.getSelectedSensorPosition()/10));
    }

    public double controlX(double setpointX) {
        double currentX = (0.54-((slideMotor.getSelectedSensorPosition()/10) * ENC_TO_METERS));
        double output = xPID.pidPosition(currentX, setpointX);
        return output;
    }

    public void moveToFront(){ //TODO: add if microSwitch
        set(DISTANCE);
    }

    public void moveTOBack(){ //TODO: add if microSwitch
        set(0);
    }
    public void set_direct(double power){
        slideMotor.set(power);
    }
}
