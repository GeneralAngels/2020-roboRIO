package frc.robot.base;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.json.JSONObject;

public class Communication extends Module {
    public Communication(Bot bobot) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable database = instance.getTable("database");
        NetworkTableEntry command = database.getEntry("command");
        command.addListener(entryNotification -> {
            try {
                bobot.handleJSON(new JSONObject(entryNotification.value.getString()));
            } catch (Exception ignored) {
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
}