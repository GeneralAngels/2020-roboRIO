package frc.robot.efrat;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.bobot.Bobot;
import frc.robot.bobot.utils.Toggle;
import frc.robot.efrat.systems.Lift;
import frc.robot.efrat.systems.Stick;
import org.json.JSONObject;

public class EfratTestingShanti extends Bobot {

    protected final String DRIVE = "drive";

    // Autonomous Sequences
    protected JSONObject robotStatus;
    // Controllers
    protected XboxController driverGamepad;
    // Systems
    protected Stick makel;
    protected Lift lift;
    protected Toggle drA, drB, drX, drY, drR, drL;

    @Override
    public void init() {
        // Controllers
        driverGamepad = new XboxController(0);
        // RGB
        makel = new Stick();
        lift = new Lift();
        // Instruction Log
        dontLogName();
        log("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        log("Efrat Testing!!!");
        log("PneumaticDrive with left stick");
        log("B for bench with x to down and y to up");
        log("A for autonomous");
        log("Press ENABLE");
        log("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        doLogName();
        // Setup Triggers
        initTriggers();
        // SuperInit -> TCP Init
        super.init();
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
        makel.set(driverGamepad.getY(GenericHID.Hand.kLeft));
        lift.set(driverGamepad.getY(GenericHID.Hand.kRight));
    }

    protected void robotStatus() {
    }

    @Override
    public JSONObject toJSON() {
        JSONObject currentJSON = super.toJSON();
        robotStatus();
        currentJSON.put(ROBOT_STATUS, robotStatus);
        return currentJSON;
    }

    @Override
    protected JSONObject handleJSON(JSONObject object) {
        return toJSON();
    }
}
