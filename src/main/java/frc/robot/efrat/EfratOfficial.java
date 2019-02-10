package frc.robot.efrat;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.bobot.Bobot;
import frc.robot.bobot.rgb.RGB;
import frc.robot.bobot.utils.Toggle;
import frc.robot.efrat.systems.*;
import frc.robot.efrat.systems.rgb.RobotIdle;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.awt.*;

import static frc.robot.bobot.drive.DifferentialDrive.OMEGA;
import static frc.robot.bobot.drive.DifferentialDrive.VELOCITY;

// TODO THIS CODE SHOULD NOT BE RAN ON THE TEST KIT! USE "EfratTesting"!
// TODO Official Efrat Code with Joysticks

/*
TODO
Robot Subsystems:
Lift
Stick & Fork
Shiri Claw & Shiri Slide
DifferentialDrive & Gears
Roller
 */

/*
TODO
canbus map
0-Right 1
1-Right 2
2-Left 1
3-Left 2
4-Shiri Slide Motor
5-Lift 1
6-Lift 2
7-Stick Motor
8-Roller Motor
 */
/*
TODO
pinmap
AI0-Lift Potentiometer
todo remove
DI2,DI3-Slide Encoder
DI4,DI5-Drive Encoder Right
DI6,DI7-Drive Encoder Left
todo remove
DI8,DI9-Stick Encoder
NavX:
todo remove
NXDI0-Lift Reset Down
NXDI1-Lift Reset Up
NXDI2-Stick Start
NXDI3-Stick End
todo combine
NXDI4-Shiri Grab N' Go 1
NXDI5-Shiri Grab N' Go 2
NXDI6-Shiri Reset Back
NXDI7-Shiri Reset Front
NXDI8-Fork Cargo Indicator
*/
/*
TODO
SolenoidMap
0,1 - Shiri
2,3 - Fork
4,5 - DriveA
6,7 - DriveB
 */
// Roller Gripper pull is right

public class EfratOfficial extends Bobot {

    protected final String DRIVE = "drive";
    protected final String GEAR = "gear";
    protected final String LIFT = "lift", TARGET_LEVEL = "target_level";

    // Autonomous Sequences
    protected JSONObject robotStatus;
    // Controllers
    protected Joystick driverLeft, driverRight;
    protected XboxController operatorGamepad;
    // Systems
    protected PneumaticDrive drive;
    protected Stick stick;
    protected Lift lift;
    protected Fork fork;
    protected Shiri shiri;
    protected Roller roller;
    protected Toggle operatorA, operatorB, operatorX, operatorY, operatorStart, operatorBack, operatorPadUp, operatorPadDown;
    protected Toggle driverLeftT, driverRightT, driverRight5, driverRight3, driverRight6, driverRight4;
    // RGB
    protected RobotIdle robotIdle;
    protected RGB rgb;
    // ControlMode
    protected boolean isAutonomous = false;
    // Compressor
    protected Compressor compressor;

    @Override
    public void init() {
        // Controllers
        driverLeft = new Joystick(0);
        driverRight = new Joystick(1);
        operatorGamepad = new XboxController(2);
        // RGB
        rgb = new RGB(69, 8);
        robotIdle = new RobotIdle();
        rgb.setPattern(robotIdle);
        // Compressor
        compressor = new Compressor(0);
        compressor.setClosedLoopControl(true);
        // Systems
//        drive = new PneumaticDrive();
        stick = new Stick();
        fork = new Fork();
        shiri = new Shiri();
        lift = new Lift();
        // Registering Subsystems
        addToJSON(drive);
        // Instruction Log
        dontLogName();
        log("///////////////////////////////////////////////////////");
        log("Efrat Official!!!");
        log("Configure Controllers!");
        log("Driver Left Port 0");
        log("Driver Right Port 1");
        log("Operator Port 2");
        log("To Begin, Press Enable");
        log("///////////////////////////////////////////////////////");
        doLogName();
        // Setup Triggers
        initTriggers();
        // SuperInit -> TCP Init
        super.init();
    }

    protected void initTriggers() {
        operatorA = new Toggle(null);
        operatorB = new Toggle(toggle -> {
            if (toggle) {
                shiri.open();
            } else {
                shiri.close();
            }
        });
        operatorX = new Toggle(null);
        operatorY = new Toggle(toggle -> {
            if (toggle) {
                fork.open();
            } else {
                fork.close();
            }
        });
        operatorBack = new Toggle(toggle -> robotIdle.color(Color.ORANGE));
        operatorStart = new Toggle(toggle -> robotIdle.color(Color.YELLOW));
        operatorPadUp = new Toggle(toggle -> lift.levelUp());
        operatorPadDown = new Toggle(toggle -> lift.levelDown());
        driverLeftT = new Toggle(toggle -> {
            if (toggle) {
                drive.gearUp();
            } else {
                drive.gearDown();
            }
        });
    }

    protected void updateTriggers() {
        if (operatorGamepad != null) {
            if (operatorA != null) operatorA.update(operatorGamepad.getAButton());
            if (operatorB != null) operatorB.update(operatorGamepad.getBButton());
            if (operatorX != null) operatorX.update(operatorGamepad.getXButton());
            if (operatorY != null) operatorY.update(operatorGamepad.getYButton());
            if (operatorStart != null) operatorStart.update(operatorGamepad.getStartButton());
            if (operatorBack != null) operatorBack.update(operatorGamepad.getBackButton());
            if (operatorPadUp != null) operatorPadUp.update(operatorGamepad.getPOV() == 0);
            if (operatorPadDown != null) operatorPadDown.update(operatorGamepad.getPOV() == 180);
        }
        if (driverLeft != null) {
            driverLeftT.update(driverLeft.getTrigger());
        }
        if (driverRight != null) {
            if (driverRightT != null) driverRightT.update(driverRight.getTrigger());
            if (driverRight3 != null) driverRight3.update(driverRight.getRawButton(3));
            if (driverRight4 != null) driverRight4.update(driverRight.getRawButton(4));
            if (driverRight5 != null) driverRight5.update(driverRight.getRawButton(5));
            if (driverRight6 != null) driverRight6.update(driverRight.getRawButton(6));
        }
    }

    private void loopSubsystems() {
        lift.loop();
    }

    @Override
    public void teleop() {
        updateTriggers();
        loopSubsystems();
        roller.set(operatorGamepad.getBumper(GenericHID.Hand.kRight) ? -0.3 : operatorGamepad.getBumper(GenericHID.Hand.kLeft) ? 0.3 : 0);
        stick.set(operatorGamepad.getY(GenericHID.Hand.kLeft) / 4.0);
        shiri.set(operatorGamepad.getY(GenericHID.Hand.kRight));
        if (!isAutonomous) {
            double[] parameters = drive.wheelsToRobot(-driverLeft.getY(), -driverRight.getY());
            drive.set(parameters[0], parameters[1]);
        }
    }

    protected void robotStatus() {
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
                if (object.has(LIFT)) {
                    JSONObject liftObject = object.getJSONObject(LIFT);
                    if (liftObject.has(TARGET_LEVEL)) {
                        lift.setTargetLevel(liftObject.getInt(TARGET_LEVEL));
                    }
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
