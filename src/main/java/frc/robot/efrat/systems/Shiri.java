package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
    private double targetX=-1;
    private double currentX = 0;
    public double y = -0.5;
    public double encoder = 0;
    public Shiri(){

        latest = this;
        hatch = new DoubleSolenoid(0, 0, 1);
//        hatch = new DoubleSolenoid(0,0, 1);
        slideMotor = new WPI_TalonSRX(14);
        //log(slideMotor.getSensorCollection().getPulseWidthPosition()+"");
        slideMotor.getSensorCollection().setQuadraturePosition(slideMotor.getSensorCollection().getPulseWidthPosition(),10);
 //       slideMotor.setInverted(true);
        slideMotor.getSensorCollection().setPulseWidthPosition(0,0);
        slideMotor.getSensorCollection().setQuadraturePosition(0,0);
//        slideMotor.getSensorCollection().setAnalogPosition(0,0);
        grab1 = new DigitalInput(PinMan.getNavDIO(1));
        grab2 = new DigitalInput(PinMan.getNavDIO(2));
        backReset = new DigitalInput(PinMan.getNavDIO(3));
        frontReset = new DigitalInput(PinMan.getNavDIO(4));
        xPID = new PID();
        xPID.setPIDF(1.7, 0.3, 0.2, 0);
    }

    public static void init() {
        if (getInstance() == null) new Shiri();
    }

    public static Shiri getInstance() {
        return latest;
    }

    public boolean isAtFront() {
        return frontReset.get();
    }

    public boolean isAtBack() {
//        log("Shiri Position-Reset");
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
//        log("open");
    }

    public void close() {
        if (hatch != null) hatch.set(DoubleSolenoid.Value.kReverse);
//        log("close");
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
//        log("Shiri: "+slideMotor.getSensorCollection().getQuadraturePosition());

    }

    public void set(double x) { //changed method
        targetX=x;
        encoder = slideMotor.getSensorCollection().getQuadraturePosition() / 10.0;
    }

    public int sign(double a){
        if (a>0)
            return 1;
        else if (a<0)  return -1;
        return 0;
    }
    public void print(){
//        log("meters: "+((-(slideMotor.getSensorCollection().getQuadraturePosition()/10.0)* ENC_TO_METERS +0.9253156529296874)));
//        log("encoder: "+((slideMotor.getSelectedSensorPosition())/10));
    }

    public double controlX(double setpointX) {
        currentX = -(slideMotor.getSensorCollection().getQuadraturePosition()/10.0)* ENC_TO_METERS +0.9253156529296874;
        double output = xPID.pidPosition(currentX, setpointX);
        return output;
    }
    // Notice! if you call loop before set, the target location will be the farthest back.
    public void loop(){
//        log("target x: "+targetX);
        currentX = -(slideMotor.getSensorCollection().getQuadraturePosition()/10.0)* ENC_TO_METERS +0.9253156529296874;
        if (targetX != -1) {
            double slideMotor_output = controlX(targetX);
            slideMotor.set(-slideMotor_output);
        }
    }

    public void moveToFront(){ //TODO: add if microSwitch
        set(DISTANCE);
    }

    public boolean in_place(double x, double diffrenceX){
        if(targetX>0.36) {
            return (currentX - (x - 0.23) > diffrenceX);
        }
        else {
            return ((x - 0.23) - currentX) > diffrenceX;
        }
    }

    public void setMotor(double d){
        slideMotor.set(d);
    }

    public void moveTOBack(){ //TODO: add if microSwitch
        set(0);
    }
    public void set_direct(double power){
        slideMotor.set(power);
    }

    public double getCurrentX(){return currentX;}
    public double getTargetX(){return targetX;}

}
