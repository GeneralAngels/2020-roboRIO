package frc.robot.yompa;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.bobot.Bobot;
import frc.robot.bobot.rgb.RGB;
import frc.robot.bobot.rgb.patterns.Rainbow;
import frc.robot.bobot.utils.PinMan;
import frc.robot.bobot.utils.Toggle;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.PneumaticDrive;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.rgb.RobotIdle;
import org.json.JSONObject;

public class Yompa extends Bobot {

    protected XboxController driverGamepad;
    // Systems
    protected RGB rgb;
    protected StickLift slift;
    protected YompaDrive drive;
    protected Toggle drA, drB, drX, drY, drR, drL;

    @Override
    public void init() {
        driverGamepad = new XboxController(0);
        slift=new StickLift();
        drive=new YompaDrive();
        rgb = new RGB(69, 8);
        rgb.setPattern(new Rainbow());
        log("YomPa Tuach INIT");
        // Setup Triggers
        initTriggers();
    }

    protected void initTriggers() {
    }

    protected void updateTriggers() {
        if (driverGamepad != null) {
            if (drA != null) drA.update(driverGamepad.getAButton());
            if (drB != null) drB.update(driverGamepad.getBButton());
            if (drX != null) drX.update(driverGamepad.getXButton());
            if (drY != null) drY.update(driverGamepad.getYButton());
            if (drR != null) drR.update(driverGamepad.getBumper(GenericHID.Hand.kRight));
            if (drL != null) drL.update(driverGamepad.getBumper(GenericHID.Hand.kLeft));
        }
    }

    @Override
    public void teleop() {
        updateTriggers();
        slift.setSpeed(driverGamepad.getY(GenericHID.Hand.kRight));
        drive.setStickNoPID(driverGamepad.getY(GenericHID.Hand.kLeft)/2,driverGamepad.getX(GenericHID.Hand.kLeft)/2);
    }
}
