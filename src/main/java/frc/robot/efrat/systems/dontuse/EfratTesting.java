package frc.robot.efrat.systems.dontuse;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.base.Bot;
import frc.robot.base.rgb.RGB;
import frc.robot.base.utils.Toggle;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Roller;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.Tomer;
import frc.robot.efrat.systems.rgb.RobotIdle;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import static frc.robot.base.drive.DifferentialDrive.OMEGA;
import static frc.robot.base.drive.DifferentialDrive.VELOCITY;

public class EfratTesting extends Bot {

    protected final String DRIVE = "drive";

    protected StateMachine stateMachine;
    // Autonomous Sequences
    protected JSONObject robotStatus;
    // Controllers
    protected XboxController driverGamepad;
    // Systems
    protected PneumaticTestingDrive pneumaticDrive;
    protected Roller roller;
    protected Toggle drA, drB, drX, drY, drR, drL;
    // RGB
    protected RobotIdle robotIdle;
    protected RGB rgb;
    // ControlMode
    protected boolean isAutonomous = false;
    protected boolean isTestBench = false;
    // Variables
    protected float testBenchSpeed = 0f;
    // NEW! GyroBitch!
    protected AHRS gyro;

    @Override
    public void init() {
        stateMachine = new StateMachine();
        // Controllers
        driverGamepad = new XboxController(0);
        // RGB
        rgb = new RGB(69, 8);
        robotIdle = new RobotIdle();
        rgb.setPattern(robotIdle);
        new Tomer();
        new Shiri();
        // Gyro
//        gyro = new AHRS(I2C.Port.kMXP);
        // Systems
        pneumaticDrive = new PneumaticTestingDrive();
        roller = new Roller();
        // Registering Subsystems
        addToJSON(pneumaticDrive);
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
        drA = new Toggle(toggle -> {
            isAutonomous = !isAutonomous;
            log("Auto " + (isAutonomous ? "Enabled" : "Disabled"));
        });
        drB = new Toggle(toggle -> {
            isTestBench = !isTestBench;
            log("Bench " + (isTestBench ? "Enabled" : "Disabled"));

        });
        drX = new Toggle(toggle -> {
            if (testBenchSpeed > -1)
                testBenchSpeed -= 0.1f;
            log("Bench Speed: " + testBenchSpeed);
        });
        drY = new Toggle(toggle -> {
            if (testBenchSpeed < 1)
                testBenchSpeed += 0.1f;
            log("Bench Speed: " + testBenchSpeed);

        });
        drR = new Toggle(null);
        drL = new Toggle(null);
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
//        updateTriggers();
//        log("Gyro " + gyro.getYaw());
        stateMachine.update(driverGamepad, null);
//        log(stateMachine.toJSON().toString());
//        double speed = -driverGamepad.getY(GenericHID.Hand.kLeft);
//        double turn = -driverGamepad.getX(GenericHID.Hand.kLeft);
        pneumaticDrive.direct(-driverGamepad.getY(GenericHID.Hand.kLeft),-driverGamepad.getY(GenericHID.Hand.kRight));
        if (!isAutonomous && !isTestBench) {
//            log("go here");
//            pneumaticDrive.set(speed, turn);
        } else if (isTestBench) {
//            pneumaticDrive.setBench(testBenchSpeed, turn);
        }
    }

    protected void robotStatus() {
        robotStatus = new JSONObject();
        JSONArray driverStatus = new JSONArray();
        JSONArray operatorStatus = new JSONArray();

        if (isAutonomous) {
            operatorStatus.put("follow_ball");
        } else if (isTestBench) {
            operatorStatus.put("testing_bench");
        } else {
            operatorStatus.put("joystick");
        }
        robotStatus.put(DRIVER_STATUS, driverStatus);
        robotStatus.put(OPERATOR_STATUS, operatorStatus);
    }

    @Override
    public JSONObject toJSON() {
        JSONObject currentJSON = super.toJSON();
        robotStatus();
        currentJSON.put(ROBOT_STATUS, robotStatus);
        return currentJSON;
    }

    @Override
    protected void handleJSON(JSONObject object) {
        JSONObject output = toJSON();
        // V and W are here:
        // pneumaticDrive -> {v, w}
        if (isAutonomous) {
            double v = 0, w = 0;
            try {
                if (object.has(DRIVE)) {
                    JSONObject driveObject = object.getJSONObject(DRIVE);
                    if (driveObject.has(VELOCITY) && driveObject.has(OMEGA)) {
                        v = object.getJSONObject(DRIVE).getFloat(VELOCITY);
                        w = object.getJSONObject(DRIVE).getFloat(OMEGA);
//                        log("AutoTCP - Good");
                        log("V " + v + " W " + w);
                    } else {
                        log("No \"v\" & \"w\" in json");
                    }
                } else {
                    log("No \"drive\" in json");
                }
            } catch (Exception e) {
                if (e instanceof JSONException) {
                    log("JSON syntax error");
                } else {
                    log("Error: " + e.toString());
                }
            }
            pneumaticDrive.setAutonomous(v, w, false);
        }
    }
}
