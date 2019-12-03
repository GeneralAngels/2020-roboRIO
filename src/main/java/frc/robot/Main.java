package frc.robot;

import com.ga2230.networking.Dialog;
import com.ga2230.networking.OnReceive;
import com.ga2230.networking.Server;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.base.Bot;
import frc.robot.bosmat.RobotC;
import frc.robot.thucc.Thucc;
import org.json.JSONObject;

public final class Main {

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

    private static class Robot extends TimedRobot {

        private Bot bobot;

        @Override
        public void robotInit() {
            bobot = new Thucc();
            bobot.init();
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


