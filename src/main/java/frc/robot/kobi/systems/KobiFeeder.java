package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.kobi.Kobi;

public class KobiFeeder extends frc.robot.base.Module {

    private static final boolean USE_MICROSWITCHES = false;

    // Slider
    private WPI_TalonSRX slider;
    private DigitalInput openSwitch, closeSwitch;

    // Feeder
    private CANSparkMax feeder;

    // Collector (Roller-Gripper)
    private WPI_TalonSRX roller;

    public KobiFeeder() {
        super("feeder");

        // Slider
        slider = new WPI_TalonSRX(16);
        closeSwitch = new DigitalInput(4);
        openSwitch = new DigitalInput(5);

        // Collector
        roller = new WPI_TalonSRX(17);
        Kobi.setupMotor(roller, FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0, 0, 0.01);

        // Feeder
        feeder = new CANSparkMax(18, CANSparkMaxLowLevel.MotorType.kBrushless);

        command("feeder", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                Direction direction;
                if (s.equals("in")) {
                    direction = Direction.In;
                } else if (s.equals("out")) {
                    direction = Direction.Out;
                } else {
                    direction = Direction.Out;
                }
                feed(direction);
                return new Tuple<>(true, "Speed set");
            }
        });

        command("slide", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                Direction direction;
                if (s.equals("in")) {
                    direction = Direction.In;
                } else if (s.equals("out")) {
                    direction = Direction.Out;
                } else {
                    direction = Direction.Out;
                }
                if (slide(direction)) {
                    return new Tuple<>(true, "Speed set");
                } else {
                    return new Tuple<>(false, "Limit-switch error");
                }
            }
        });

        command("roll", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                Direction direction;
                if (s.equals("in")) {
                    direction = Direction.In;
                } else if (s.equals("out")) {
                    direction = Direction.Out;
                } else {
                    direction = Direction.Out;
                }
                roll(direction);
                return new Tuple<>(true, "Speed set");
            }
        });
    }

    public void roll(Direction direction) {
        if (direction == Direction.Stop) {
            roller.set(0);
        } else {
            if (direction == Direction.In) {
                roller.set(0.5);
            } else {
                roller.set(-0.5);
            }
        }
    }

    public void feed(Direction direction) {
        if (direction == Direction.Stop) {
            feeder.set(0);
        } else {
            if (direction == Direction.In) {
                feeder.set(-1);
            } else {
                feeder.set(1);
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
