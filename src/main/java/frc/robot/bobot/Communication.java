package frc.robot.bobot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.json.JSONObject;

public class Communication extends Subsystem {
    public Communication(Bobot bobot) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable database = instance.getTable("database");
        NetworkTableEntry command = database.getEntry("command");
        command.addListener(entryNotification -> {
            bobot.handleJSON(new JSONObject(entryNotification.value.getString()));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
}