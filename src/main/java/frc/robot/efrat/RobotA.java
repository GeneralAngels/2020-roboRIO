package frc.robot.efrat;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.bobot.Bobot;
import frc.robot.bobot.rgb.RGB;
import frc.robot.bobot.utils.Toggle;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Klein;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.Tomer;
import frc.robot.efrat.systems.rgb.RobotIdle;
import frc.robot.efrat.systems.robota.RobotADrive;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import static frc.robot.bobot.drive.DifferentialDrive.OMEGA;
import static frc.robot.bobot.drive.DifferentialDrive.VELOCITY;

@SuppressWarnings("ALL")
public class RobotA extends Bobot {
    private boolean isAutonomous = false;
    // Joysticks
    private Joystick driverLeft, driverRight;
    private XboxController operatorGamepad;
    // Systems
    private RobotADrive drive;
    private Tomer tomer;
    private Shiri shiri;
    private Shanti shanti;
    private Klein klein;
    // Toggles
    private Toggle operatorA, operatorB, operatorX, operatorY, operatorStart, operatorBack, operatorPadUp, operatorPadDown, driverLeftT, driverRightT, driverRight5, driverRight3, driverRight6, driverRight4;
    // RGB
    private RobotIdle robotIdle;
    private RGB rgb;
    // Compressor
    private Compressor compressor;
    // StateMachine
    private StateMachine stateMachine;
    private JSONObject robotStatus;

    // Code
    @Override
    public void init() {
        initControllers();
        initRGB();
        initSystems();
//        initStateMachine();
//        initCompressor();
        initTriggers();
        super.init();
        instructions();
    }

    private void initCompressor() {
        compressor = new Compressor(0);
        compressor.setClosedLoopControl(true);
    }

    private void initStateMachine() {
        stateMachine = new StateMachine();
        addToJSON(stateMachine);
    }

    private void initControllers() {
        driverLeft = new Joystick(0);
        driverRight = new Joystick(1);
        operatorGamepad = new XboxController(2);
    }

    private void instructions() {
        dontLogName();
        log("=======================================================");
        log("Efrat A");
        log("Configure Controllers!");
        log("Driver Left Port 0");
        log("Driver Right Port 1");
        log("Operator Port 2");
        log("To Begin, Press Enable");
        log("=======================================================");
        doLogName();
    }

    private void initRGB() {
        rgb = new RGB(69, 8);
        robotIdle = new RobotIdle();
        rgb.setPattern(robotIdle);
    }

    private void initSystems() {
        drive = new RobotADrive();
//        tomer = new Tomer();
//        shiri = new Shiri();
        shanti = new Shanti();
//        klein = new Klein();
        addToJSON(drive);
    }

    private void initTriggers() {
        driverLeftT = new Toggle(new Toggle.Change() {
            @Override
            public void change(boolean toggle) {
                isAutonomous = toggle;
            }
        });
        driverRightT = new Toggle(new Toggle.Change() {
            @Override
            public void change(boolean toggle) {
                if (toggle) {
                    drive.gearUp();
                } else {
                    drive.gearDown();
                }
            }
        });
        operatorBack = new Toggle(new Toggle.Change() {
            @Override
            public void change(boolean toggle) {
                if (toggle) drive.gearDown();
                if (!toggle) drive.gearUp();
//               if(toggle)shiri.open();
//               if(!toggle)shiri.close();
            }
        });
    }

    private void updateTriggers() {
        if (driverLeft != null) {
            if (driverLeftT != null) driverLeftT.update(driverLeft.getTrigger());
        }
        if (driverRight != null) {
            if (driverRightT != null) driverRightT.update(driverRight.getTrigger());
            if (driverRight3 != null) driverRight3.update(driverRight.getRawButton(3));
            if (driverRight4 != null) driverRight4.update(driverRight.getRawButton(4));
            if (driverRight5 != null) driverRight5.update(driverRight.getRawButton(5));
            if (driverRight6 != null) driverRight6.update(driverRight.getRawButton(6));
        }
        if (operatorGamepad != null) {
            if (operatorBack != null) operatorBack.update(operatorGamepad.getBackButton());
            if (operatorStart != null) operatorStart.update(operatorGamepad.getStartButton());
            if (operatorA != null) operatorA.update(operatorGamepad.getAButton());
            if (operatorB != null) operatorB.update(operatorGamepad.getBButton());
            if (operatorX != null) operatorX.update(operatorGamepad.getXButton());
            if (operatorY != null) operatorA.update(operatorGamepad.getYButton());
        }
    }

    private void loopSubsystems() {
    }

    @Override
    public void autonomous(){
        teleop();
    }

    @Override
    public void teleop() {
        updateTriggers();
        loopSubsystems();
//        stateMachine.update(operatorGamepad, null);

        if (!isAutonomous) {
            // Klein Drive
//            double[] parameters = drive.wheelsToRobot(-driverLeft.getY(), -driverRight.getY());
//            drive.set(parameters[0], parameters[1]);
            // Klein Drive
//            shanti.setStick(operatorGamepad.getX(GenericHID.Hand.kRight));
//            shiri.setMotor(operatorGamepad.getX(GenericHID.Hand.kRight));
//            shanti.setLift(operatorGamepad.getY(GenericHID.Hand.kRight));
//            shiri.print();
//            shiri.set(0.35);
//            shiri.loop();
            shanti.set(0.5, 0);
            shanti.loop();
//              shanti.setLift(0.2);
            shanti.print();
//            klein.set(operatorGamepad.getX(GenericHID.Hand.kRight));
//            drive.setStickNoPID(operatorGamepad.getY(GenericHID.Hand.kLeft), operatorGamepad.getX(GenericHID.Hand.kLeft));
        }
    }

    private void robotStatus() {
        robotStatus = new JSONObject();
        JSONArray driverStatus = new JSONArray();
        JSONArray operatorStatus = new JSONArray();
        if (isAutonomous) {
            operatorStatus.put("follow_ball");
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
    protected JSONObject handleJSON(JSONObject object) {
        JSONObject output = toJSON();
        // V and W are here:
        // pneumaticDrive -> {v, w}
        if (isAutonomous) {
            double v = 0, w = 0;
            try {
                String DRIVE = "drive";
                if (object.has(DRIVE)) {
                    JSONObject driveObject = object.getJSONObject(DRIVE);
                    if (driveObject.has(VELOCITY) && driveObject.has(OMEGA)) {
                        v = driveObject.getFloat(VELOCITY);
                        w = driveObject.getFloat(OMEGA);
//                        log("AutoTCP - Good");
                        log("V " + v + " W " + w);
                    } else {
                        log("No \"v\" & \"w\" in json");
                    }
                    String GEAR = "gear";
                    if (driveObject.has(GEAR)) {
                        if (driveObject.getBoolean(GEAR)) {
                            drive.gearUp();
                        } else {
                            drive.gearDown();
                        }
                    }
                } else {
                    log("No \"DifferentialDrive\" in json");
                }
            } catch (Exception e) {
                if (e instanceof JSONException) {
                    log("JSON syntax error");
                } else {
                    log("Error: " + e.toString());
                }
            }
            drive.setAutonomous(v, w);
        }
        return output;
    }
}