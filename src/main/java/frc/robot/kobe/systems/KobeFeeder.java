package frc.robot.kobe.systems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ga2230.shleam.advanced.frc.FRCModule;
import com.ga2230.shleam.base.structure.Function;
import com.ga2230.shleam.base.structure.Result;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.base.utils.General;

public class KobeFeeder extends FRCModule {

    private static final boolean USE_MICROSWITCHES = false;

    private static final double FEEDER_MAX_CURRENT_DERIVATIVE = 1;

    private double lastCurrent = 0;
    private double currentCurrent = 0;
    private long time = 0;

    // Slider
    private WPI_TalonSRX slider;
    private DigitalInput openSwitch, closeSwitch;

    // Feeder
    private CANSparkMax feeder;

    // Collector (Roller-Gripper)
    private WPI_TalonSRX roller;

    public KobeFeeder() {
        super("feeder");

        // Slider
        slider = new WPI_TalonSRX(16);
        closeSwitch = new DigitalInput(4);
        openSwitch = new DigitalInput(5);

        // Collector
        roller = new WPI_TalonSRX(17);
        General.setupMotor(roller, FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0, 0, 0.01);

        // Feeder
        feeder = new CANSparkMax(18, CANSparkMaxLowLevel.MotorType.kBrushless);

        register("feed", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                feed(General.fromString(parameter));
                return Result.finished("Set");
            }
        });

        register("slide", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                if (slide(General.fromString(parameter))) {
                    return Result.finished("Set");
                } else {
                    return Result.notFinished("Limit-switch error");
                }
            }
        });

        register("roll", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                roll(General.fromString(parameter), true);
                return Result.finished("Set");
            }
        });
    }

    public void roll(Direction direction, boolean fast) {
        if (direction == Direction.Stop) {
            roller.set(0);
        } else {
            double speed = fast ? 0.75 : 0.5;
            if (direction == Direction.In) {
                roller.set(speed);
            } else {
                roller.set(-speed);
            }
        }
    }

    public void feed(Direction direction) {
        // Switch errors
        lastCurrent = currentCurrent;
        currentCurrent = feeder.getOutputCurrent();
        // Calculate current limit
        long delta = time - millis();
        boolean overCurrent = delta == 0 || Math.abs((currentCurrent - lastCurrent) / delta) > FEEDER_MAX_CURRENT_DERIVATIVE;
        set("test-current", String.valueOf(currentCurrent));
        if (direction == Direction.Stop || overCurrent) {
            feeder.set(0);
        } else {
            if (direction == Direction.In) {
                feeder.set(-0.25);
            } else {
                feeder.set(0.25);
            }
        }
    }

    public void limitSwitchTest() {
        set("min_sw", String.valueOf(closeSwitch.get()));
        set("max_sw", String.valueOf(openSwitch.get()));
    }

    public boolean slide(Direction direction) {
        if (direction == Direction.Stop) {
            slider.set(0);
            return true;
        } else {
            if (direction == Direction.In) {
                // if (!closeSwitch.get()) { //microswitch isn't working
                // Todo m/s l/s
                if (closeSwitch.get() && USE_MICROSWITCHES) {
                    slider.set(0);
                    return true;
                } else {
                    slider.set(0.2);
                    return false;
                }
            } else {
                if (openSwitch.get() && USE_MICROSWITCHES) {
                    slider.set(0);
                    return true;
                } else {
                    slider.set(-0.2);
                    return false;
                }
            }
        }
    }

    public enum Direction {
        In,
        Out,
        Stop
    }
}
