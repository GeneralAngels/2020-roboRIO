package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.bobot.Bobot;
import frc.robot.efrat.EfratTestingFull;
import frc.robot.efrat.RobotA;

public class Robot extends TimedRobot {
    private Bobot bobot;

    @Override
    public void robotInit() {
        bobot = new RobotA();
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