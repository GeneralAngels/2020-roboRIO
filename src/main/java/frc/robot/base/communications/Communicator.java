package frc.robot.base.communications;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.base.Bot;
import frc.robot.base.Module;
import org.json.JSONObject;

public class Communicator extends Module {

    // NetworkTables things
    private NetworkTableInstance instance;
    private NetworkTable database;
    private NetworkTableEntry push, pull;
    // Bot
    private Bot bot;
    // Previous parameters
    private String lastPush = null;
    private String lastPull = null;
    // Current parameters
    private String currentPush = null;
    private String currentPull = null;

    public Communicator(Bot bot) {
        this.bot = bot;
        // Init NetworkTables
        this.instance = NetworkTableInstance.getDefault();
        this.database = this.instance.getTable("database");
        this.push = this.database.getEntry("push");
        this.pull = this.database.getEntry("pull");
    }

    private void initListener() {
        this.push.addListener(entryNotification -> {
            if (this.bot != null)
                this.bot.pushJSON(new JSONObject(entryNotification.value.getString()));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public void update() {
        if (this.bot != null) {
            currentPush = this.push.getValue().getString();
            if (!currentPush.equals(lastPush)) {
                this.bot.pushJSON(new JSONObject(currentPush));
            }
            currentPush = null;
            currentPull = this.bot.pullJSON().toString();
            if (!currentPull.equals(lastPull)) {
                this.pull.setString(currentPull);
            }
            currentPull = null;
        }
    }
}