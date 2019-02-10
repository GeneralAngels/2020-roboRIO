package frc.robot.efrat;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.bobot.Bobot;
import frc.robot.bobot.utils.PinManager;
import frc.robot.bobot.utils.Toggle;
import frc.robot.efrat.systems.Lift;
import frc.robot.efrat.systems.PneumaticDrive;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.Stick;
import org.json.JSONObject;

public class EfratTestingFull extends Bobot {

    protected final String DRIVE = "drive";

    // Autonomous Sequences
    protected JSONObject robotStatus;
    // Controllers
    protected XboxController driverGamepad;
    // Systems
    protected Stick makel;
    protected Lift lift;
    protected Shiri shiri;
    protected PneumaticDrive drive;
    protected Toggle drA, drB, drX, drY, drR, drL;
    protected Compressor compressor = new Compressor(0);

    @Override
    public void init() {
        // Controllers
        compressor.setClosedLoopControl(true);
        driverGamepad = new XboxController(0);
        makel = new Stick();
        lift = new Lift();
        shiri = new Shiri();
        drive = new PneumaticDrive();
        PinManager pinManager = new PinManager();
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
        drB = new Toggle(toggle -> {
            if (toggle) drive.gearUp();
            else drive.gearDown();
        });
        drX = new Toggle(toggle -> {
            if (toggle) shiri.open();
            else shiri.close();
        });
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
        double divide = 2.0;
        drive.setStickNoPID(-driverGamepad.getY(GenericHID.Hand.kLeft) / divide, -driverGamepad.getX(GenericHID.Hand.kLeft) / divide);
//        makel.set(-driverGamepad.getX(GenericHID.Hand.kRight));
//        lift.set(driverGamepad.getY(GenericHID.Hand.kRight));
//        log(Double.toString(-(driverGamepad.getTriggerAxis(GenericHID.Hand.kRight) - driverGamepad.getTriggerAxis(GenericHID.Hand.kLeft))));
//        shiri.set(-(driverGamepad.getTriggerAxis(GenericHID.Hand.kRight) - driverGamepad.getTriggerAxis(GenericHID.Hand.kLeft)));

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
