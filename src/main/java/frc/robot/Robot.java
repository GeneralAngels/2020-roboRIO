package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.base.Bot;
import frc.robot.efrat.RobotC;

public class Robot extends TimedRobot {
    private Bot bobot;

    @Override
    public void robotInit() {
        bobot = new RobotC();
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