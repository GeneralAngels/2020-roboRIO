package frc.robot.efrat.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.bobot.Subsystem;
import frc.robot.bobot.utils.PinMan;

public class Shiri extends Subsystem {

    private static Shiri latest;
    public DigitalInput frontReset, backReset, grab1, grab2;
    private DoubleSolenoid hatch;
    private WPI_TalonSRX slideMotor;

    public Shiri() {
        latest = this;
        hatch = new DoubleSolenoid(0, 1);
        slideMotor = new WPI_TalonSRX(14);
        grab1 = new DigitalInput(PinMan.getNavDIO(1));
        grab2 = new DigitalInput(PinMan.getNavDIO(2));
        backReset = new DigitalInput(PinMan.getNavDIO(3));
        frontReset = new DigitalInput(PinMan.getNavDIO(4));
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

    public void moveToFront() {

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

    public void set(double speed) {
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
}
