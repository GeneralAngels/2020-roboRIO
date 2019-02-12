package frc.robot.efrat;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import frc.robot.bobot.Bobot;
import frc.robot.bobot.rgb.RGB;
import frc.robot.bobot.utils.PinManager;
import frc.robot.bobot.utils.Toggle;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.*;
import frc.robot.efrat.systems.rgb.RobotIdle;
import org.json.JSONObject;

public class EfratTestingFull extends Bobot {

    protected final String DRIVE = "drive";

    // Autonomous Sequences
    protected JSONObject robotStatus;
    // Controllers
    protected XboxController driverGamepad;
    // Systems
    protected RGB rgb;
    protected StateMachine stateMachine;
    protected Stick makel;
    protected Shiri shiri;
//    protected Shanti shanti;
    protected PneumaticDrive drive;
    protected Toggle drA, drB, drX, drY, drR, drL;
    protected Compressor compressor = new Compressor(0);
    protected AHRS gyroBitch;
    protected Stick stick;

    @Override
    public void init() {
        // StateMachine
        stateMachine = new StateMachine();
        // Controllers
        gyroBitch = new AHRS(I2C.Port.kMXP);

//        shanti = new Shanti();

        compressor.setClosedLoopControl(true);
        driverGamepad = new XboxController(0);
//        makel = new Stick();
//        lift = new Lift();
        shiri = new Shiri();
//        drive = new PneumaticDrive();
//        stick = new Stick();
        addToJSON(stateMachine);
        PinManager pinManager = new PinManager();
        rgb = new RGB(69, 8);
        rgb.setPattern(new RobotIdle());
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
//            if (toggle) drive.gearUp();
//            else drive.gearDown();
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
        stateMachine.update(driverGamepad, null);
//        shanti.set(0.3,0);
 //       shanti.print();
//        log("Byro, Gitch: " + gyroBitch.getYaw());
        double divide = 2.0;
//        drive.setBench(driverGamepad.getY(GenericHID.Hand.kLeft)/divide, driverGamepad.getX(GenericHID.Hand.kLeft)/divide);
//        drive.direct(driverGamepad.getY(GenericHID.Hand.kLeft)+driverGamepad.getX(GenericHID.Hand.kLeft),driverGamepad.getY(GenericHID.Hand.kLeft)-driverGamepad.getX(GenericHID.Hand.kLeft));
 //       drive.set(-driverGamepad.getY(GenericHID.Hand.kLeft) / divide, -driverGamepad.getX(GenericHID.Hand.kLeft) / divide);
 //       drive.setStickNoPID(-driverGamepad.getY(GenericHID.Hand.kLeft) / divide, -driverGamepad.getX(GenericHID.Hand.kLeft) / divide);
//        stick.set(-driverGamepad.getX(GenericHID.Hand.kRight));
        //        makel.set(-driverGamepad.getX(GenericHID.Hand.kRight));
//        log(Double.toString(-(driverGamepad.getTriggerAxis(GenericHID.Hand.kRight) - driverGamepad.getTriggerAxis(GenericHID.Hand.kLeft))));
//        log(Double.toString(-(driverGamepad.getTriggerAxis(GenericHID.Hand.kRight) - driverGamepad.getTriggerAxis(GenericHID.Hand.kLeft))));

        shiri.set(-(driverGamepad.getTriggerAxis(GenericHID.Hand.kRight) - driverGamepad.getTriggerAxis(GenericHID.Hand.kLeft)));

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
