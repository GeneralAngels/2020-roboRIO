package frc.robot.myrobot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.base.Bot;
import frc.robot.base.Module;
import frc.robot.base.utils.Toggle;
import org.json.JSONObject;

public class MyRobot extends Bot {

    private MyClaw mMyClaw = null;
    private Toggle mToggle = null;
    private XboxController mController = null;

    @Override
    public void init() {
        mController = new XboxController(0); // Initialize controller
        mMyClaw = new MyClaw(); // Initialise MyClaw
        mToggle = new Toggle(state -> {
            if (state) {
                mMyClaw.open();
            } else {
                mMyClaw.close();
            }
        }); // Initialize toggle
        register(mMyClaw); // Register mMyClaw as a submodule
        super.init(); // Initialize Comms
    }

    @Override
    public void autonomous() {
        teleop(); // Redirect autonomous to teleop because there is no autonomous period in 'Destination: Deep Space'
    }

    @Override
    public void teleop() {
        mToggle.update(mController.getXButton()); // Update toggle's input
        super.teleop(); // Update communication
    }

    private class MyClaw extends Module {

        private DoubleSolenoid mDSol = null;

        public MyClaw() {
            mDSol = new DoubleSolenoid(0, 0, 1); // Initialize mDSol at CAN 0, Pins (0,1)
        }

        public void open() {
            mDSol.set(DoubleSolenoid.Value.kForward); // Open piston
        }

        public void close() {
            mDSol.set(DoubleSolenoid.Value.kReverse); // Close piston
        }

        @Override
        public JSONObject pullJSON() {
            JSONObject sJSON = super.pullJSON(); // Pull JSON from super (enumerate submodules)
            sJSON.put("state", mDSol.get() == DoubleSolenoid.Value.kForward); // Add mDSol state to the JSON as a boolean
            return sJSON; // Return the combined JSON
        }

        @Override
        public void pushJSON(JSONObject object) {
            if (object.has("state")) { // Check if there's a command
                boolean state = object.getBoolean("state"); // Get the requested piston position
                if (state) {
                    open(); // Open piston
                } else {
                    close(); // Close piston
                }
            }
            super.pushJSON(object); // Execute pushJSON on super (enumerate submodules)
        }
    }

}