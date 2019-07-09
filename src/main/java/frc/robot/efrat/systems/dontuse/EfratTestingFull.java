package frc.robot.efrat.systems.dontuse;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.base.Bot;
import frc.robot.base.rgb.RGB;
import frc.robot.base.utils.Toggle;
import frc.robot.efrat.statemachine.StateMachine;
import frc.robot.efrat.systems.Shanti;
import frc.robot.efrat.systems.Shiri;
import frc.robot.efrat.systems.rgb.RobotIdle;
import org.json.JSONObject;

public class EfratTestingFull extends Bot {

    protected final String DRIVE = "drive";

    // Autonomous Sequences
    protected JSONObject robotStatus;
    // Controllers
    protected XboxController driverGamepad;
    // Systems
    protected RGB rgb;
    protected StateMachine stateMachine;
    protected Shiri shiri;
    protected Shanti shanti;
    protected PneumaticDrive drive;
    protected Toggle drA, drB, drX, drY, drR, drL;
//    protected Compressor compressor = new Compressor(0);
    protected AHRS gyroBitch;
    protected WPI_TalonSRX slideMotor;
    protected double testing =  0;
    protected double counter=0;
    protected PneumaticDrive pneumaticDrive;

    @Override
    public void init() {
        // StateMachine
        // Controllers
        gyroBitch = new AHRS(I2C.Port.kMXP);

//        shanti = new Shanti();
//        shiri = new Shiri();
//        compressor.setClosedLoopControl(true);

        pneumaticDrive = new PneumaticDrive();
        driverGamepad = new XboxController(0);
//        slideMotor = new WPI_TalonSRX(14);
        addToJSON(stateMachine);
        PinMan pinManager = new PinMan();
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
        stateMachine = new StateMachine(false);

    }

    private void loopSystems(){
        shiri.loop();
        shanti.loop();
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
        drA = new Toggle(toggle -> {testing+=0.1;
        //log("testing:" + testing);
        });
        drY = new Toggle(toggle -> {testing-=0.1;
            //log("testing:" + testing);
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

    private long progress=0;

    @Override
    public void teleop() {
//        updateTriggers();
//        if (counter > 50) {
//            stateMachine.update(driverGamepad, null);
//        }
//        shanti.applyPower(0.5,0.5);
//        shanti.print();
//        shiri.applyPower(0);
//        shiri.set_direct(driverGamepad.getY(GenericHID.Hand.kLeft));
//        shiri.print();
//        loopSystems();
//        counter+=1;
//        pneumaticDrive.applyPower(0,0);
        log("v:" + driverGamepad.getY(GenericHID.Hand.kLeft)+",w:" + driverGamepad.getX(GenericHID.Hand.kLeft));
        pneumaticDrive.set(driverGamepad.getY(GenericHID.Hand.kLeft), driverGamepad.getX(GenericHID.Hand.kLeft), false);
        progress++;
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


}
