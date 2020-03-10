package frc.robot;

import com.ga2230.shleam.advanced.frc.FRCRobot;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.kobe.Kobe;

public final class Main {

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

    private static class Robot extends TimedRobot {

        private FRCRobot robot;

        @Override
        public void robotInit() {
            robot = new Kobe();
        }

        @Override
        public void autonomousInit() {
            robot.autonomousSetup();
        }

        @Override
        public void autonomousPeriodic() {
            robot.autonomousLoop();
        }

        @Override
        public void teleopInit() {
            robot.teleopSetup();
        }

        @Override
        public void teleopPeriodic() {
            robot.teleopLoop();
        }
    }
}


