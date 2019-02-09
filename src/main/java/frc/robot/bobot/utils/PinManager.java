package frc.robot.bobot.utils;


import frc.robot.bobot.Subsystem;

public class PinManager extends Subsystem {
    private static final int NAVX_DIO = 9;
    private static final int NAVX_AIN = 3;
    private static final int RIO_AOUT = 1;
    private static final int RIO_DIO = 10;
    private static final int RIO_PWM = 10;
    private static final int RIO_AIN = 4;

    public int getPWM(int pin) {
        if (pin <= NAVX_DIO) {
            return pin + RIO_PWM;
        }
        log("PWM Pin Out Of Range!");
        return 0;
    }

    public int getDIO(int pin) {
        if (pin <= NAVX_DIO) {
            return pin + RIO_DIO + (pin > 3 ? 4 : 0);
        }
        log("DIO Pin Out Of Range!");
        return 0;
    }

    public int getAIN(int pin) {
        if (pin <= NAVX_AIN) {
            return pin + RIO_AIN;
        }
        log("AIN Pin Out Of Range!");
        return 0;
    }

    public int getAOUT(int pin) {
        if (pin <= RIO_AOUT) {
            return pin;
        }
        log("AIN Pin Out Of Range!");
        return 0;
    }
}
