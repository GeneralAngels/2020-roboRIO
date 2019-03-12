package frc.robot.bobot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.json.JSONObject;

public class Communication extends Subsystem {
    public Communication(Bobot bobot) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
//        instance.setUpdateRate(0.02);
        NetworkTable database = instance.getTable("database");
        NetworkTableEntry command = database.getEntry("command");
        command.addListener(entryNotification -> {
            try {
//                log(entryNotification.value.getString());
                bobot.handleJSON(new JSONObject(entryNotification.value.getString()));
            } catch (Exception ignored) {
                //log("hello", "hello");
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
}