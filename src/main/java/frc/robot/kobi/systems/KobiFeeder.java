package frc.robot.kobi.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;

public class KobiFeeder extends frc.robot.base.Module {

    // Slider
    private WPI_TalonSRX slider;
    private DigitalInput maximumSwitch1, maximumSwitch2, minimumSwitch1, minimumSwitch2;

    // Feeder
    private CANSparkMax feeder;

    // Collector (Roller-Gripper)
    private WPI_TalonSRX roller;

    public KobiFeeder() {
        super("feeder");

        // Slider
        slider = new WPI_TalonSRX(16);
        minimumSwitch1 = new DigitalInput(9);
        minimumSwitch2 = new DigitalInput(8);
        maximumSwitch1 = new DigitalInput(7);
        maximumSwitch2 = new DigitalInput(6);

        // Collector
        roller = new WPI_TalonSRX(17);

        // Feeder
        feeder = new CANSparkMax(18, CANSparkMaxLowLevel.MotorType.kBrushless);

        command("collector", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (s.equals("in")) {
                    rollIn();
                    return new Tuple<>(true, "Collecting");
                } else if (s.equals("out")) {
                    rollOut();
                    return new Tuple<>(true, "Collecting");
                }
                return new Tuple<>(false, "Wrong parameter");
            }
        });

        command("feeder", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (s.equals("in")) {
                    feedIn();
                } else if (s.equals("out")) {
                    feedOut();
                } else {
                    feedStop();
                }
                return new Tuple<>(true, "Speed set");
            }
        });

        command("slide", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                boolean result = false;
                if (s.equals("in")) {
                    result = slideIn();
                } else if (s.equals("out")) {
                    result = slideOut();
                } else {
                    result = slideStop();
                }
                if (result) {
                    return new Tuple<>(true, "Speed set");
                } else {
                    return new Tuple<>(false, "Limit-switch error");
                }
            }
        });

        command("roll", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (s.equals("in")) {
                    rollIn();
                } else if (s.equals("out")) {
                    rollOut();
                } else {
                    rollStop();
                }
                return new Tuple<>(true, "Speed set");
            }
        });
    }

    public void rollIn() {
        roller.set(0.2);
    }

    public void rollOut() {
        roller.set(-0.2);
    }

    public void rollStop() {
        roller.set(0);
    }

    public void feedIn() {
        feeder.set(-0.5);
    }

    public void feedOut() {
        feeder.set(0.5);
    }

    public void feedStop() {
        feeder.set(0);
    }

    public boolean slideIn() {
        if (minimumSwitch1.get() || minimumSwitch2.get()) {
            slider.set(0);
            return true;
        } else {
            slider.set(-0.2);
            return false;
        }
    }

    public boolean slideOut() {
        if (maximumSwitch1.get() || maximumSwitch1.get()) {
            slider.set(0);
            return true;
        } else {
            slider.set(0.2);
            return false;
        }
    }

    public boolean slideStop() {
        slider.set(0);
        return true;
    }
}
