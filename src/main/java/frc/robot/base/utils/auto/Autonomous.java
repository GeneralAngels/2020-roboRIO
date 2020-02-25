package frc.robot.base.utils.auto;

import frc.robot.base.Bot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Base64;
import java.util.zip.CRC32;

public class Autonomous extends frc.robot.base.Module {

    private ArrayList<String> currentlyAwaiting = new ArrayList<>();
    private ArrayList<String> currentlyAsync = new ArrayList<>();
    private ArrayList<String> currentlyDone = new ArrayList<>();

    private Bot bot;

    public Autonomous(Bot bot) {
        super("autonomous");
        this.bot = bot;

        command("load", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                CRC32 crc = new CRC32();
                crc.update(s.getBytes());
                // Parse auto
                String decoded = new String(Base64.getDecoder().decode(s));
                // Clear
                currentlyAwaiting.clear();
                currentlyAsync.clear();
                currentlyDone.clear();
                // Add
                currentlyAwaiting.addAll(Arrays.asList(decoded.split(",")));
                // Set state
                set("state", "loaded");
                return new Tuple<>(true, String.valueOf(crc.getValue()));
            }
        });
    }

    public void loop() {
        try {
            if (currentlyAwaiting.size() > 0) {
                // Parse
                String currentAwaiting = currentlyAwaiting.get(0);
                if (currentAwaiting.length() > 0 && !currentAwaiting.startsWith("//")) {
                    // Update set
                    set("current_block", currentAwaiting);
                    // Determine type
                    boolean isAsync = currentAwaiting.startsWith("a");
                    // Run
                    if (isAsync) {
                        // Move to currently async
                        currentlyAsync.add(currentlyAwaiting.remove(0));
                    } else {
                        if (executeCommand(currentAwaiting)) {
                            // Move to done
                            currentlyDone.add(currentlyAwaiting.remove(0));
                        }
                    }
                } else {
                    currentlyAwaiting.remove(0);
                }
            }
            if (currentlyAsync.size() > 0) {
                // Run all async
                for (int i = 0; i < currentlyAsync.size(); i++) {
                    if (executeCommand(currentlyAsync.get(i))) {
                        // Move to done
                        currentlyDone.add(currentlyAsync.remove(i));
                    }
                }
                // List all
                StringBuilder builder = new StringBuilder();
                for (int i = 0; i < currentlyAsync.size(); i++) {
                    if (builder.length() > 0)
                        builder.append(",");
                    builder.append(currentlyAsync.get(i));
                }
                set("current_async", builder.toString());
            }
            // Set state
            set("state", currentlyAsync.size() == 0 && currentlyAwaiting.size() == 0 ? "done" : "running");
        } catch (Exception ignored) {
            set("last-error", ignored.toString());
        }
    }

    public void add(String command) {
        currentlyAwaiting.add(command);
    }

    private boolean executeCommand(String string) {
        // Divider
        String divider = " ";
        // Run command and wait for result
        // a/b master command param
        try {
            String[] typeSplit = string.split(divider, 2);
            String[] masterSplit = typeSplit[1].split(divider, 2);
            String[] commandSplit = masterSplit[1].split(divider, 2);
            Tuple<Boolean, String> result = bot.find(masterSplit[0]).execute(commandSplit[0], commandSplit.length > 1 ? commandSplit[1] : null);
            // Return done
            return result == null || result.getA();
        } catch (Exception ignored) {
            // Return done
            return true;
        }
    }
}
