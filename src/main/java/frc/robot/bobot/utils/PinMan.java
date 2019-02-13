package frc.robot.bobot.utils;


public class PinMan {
    private static final int NAVX_DIO = 9;
    private static final int NAVX_AIN = 3;
    private static final int RIO_AOUT = 1;
    private static final int RIO_DIO = 10;
    private static final int RIO_PWM = 10;
    private static final int RIO_AIN = 4;

    public static int getNavPWM(int pin) {
        if (pin <= NAVX_DIO) {
            return pin + RIO_PWM;
        }
        System.out.println("PWM Pin Out Of Range!");
        return 0;
    }

    public static int getNavDIO(int pin) {
        if (pin <= NAVX_DIO) {
            return pin + RIO_DIO + (pin > 3 ? 4 : 0);
        }
        System.out.println("DIO Pin Out Of Range!");
        return 0;
    }

    public static int getNavAIN(int pin) {
        if (pin <= NAVX_AIN) {
            return pin + RIO_AIN;
        }
        System.out.println("AIN Pin Out Of Range!");
        return 0;
    }

    public static int getNavAOUT(int pin) {
        if (pin <= RIO_AOUT) {
            return pin;
        }
        System.out.println("AOUT Pin Out Of Range!");
        return 0;
    }
}
