package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.base.Bot;
import frc.robot.base.communication.Networking;
import frc.robot.bosmat.RobotC;

import java.util.function.Supplier;

public final class Main {

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

    private static class Robot extends TimedRobot {

        private Bot bobot;

        @Override
        public void robotInit() {
            bobot = new RobotC();
            bobot.init();
            Networking.setBot(bobot);
        }

        @Override
        public void autonomousInit() {
        }

        @Override
        public void autonomousPeriodic() {
            bobot.autonomous();
        }

        @Override
        public void teleopPeriodic() {
            bobot.teleop();
        }

        @Override
        public void testPeriodic() {
        }
    }
}


