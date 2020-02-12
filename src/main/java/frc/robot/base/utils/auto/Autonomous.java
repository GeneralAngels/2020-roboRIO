package frc.robot.base.utils.auto;

import frc.robot.base.Bot;

import java.util.ArrayList;

public class Autonomous extends frc.robot.base.Module {

    private ArrayList<String> currentlyAwaiting = new ArrayList<>();
    private ArrayList<String> currentlyAsync = new ArrayList<>();
    private ArrayList<String> currentlyDone = new ArrayList<>();

    private Bot bot;

    public Autonomous(Bot bot) {
        super("autonomous");
        this.bot = bot;

        // TODO only for testing
        currentlyAwaiting.add("b follower create 10 10");
        currentlyAwaiting.add("a follower follow");
    }

    public void loop() {
        if (currentlyAwaiting.size() > 0) {
            // Parse
            String currentAwaiting = currentlyAwaiting.get(0);
            String[] currentAwaitingSplit = currentAwaiting.split(" ", 2);
            String type = currentAwaitingSplit[0];
            String command = currentAwaitingSplit[1];
            // Update set
            set("current_block", command);
            // Determine type
            boolean isAsync = type.equals("a");
            // Run
            if (isAsync) {
                // Move to currently async
                currentlyAsync.add(currentlyAwaiting.remove(0));
            } else {
                if (executeCommand(command)) {
                    // Move to done
                    currentlyDone.add(currentlyAwaiting.remove(0));
                }
            }

        }
        if (currentlyAsync.size() > 0) {
            // Run all async
            for (String string : currentlyAsync) {
                if (executeCommand(string)) {
                    // Move to done
                    currentlyAsync.remove(string);
                    currentlyDone.add(string);
                }
            }
        }
        // Set state
        set("state", currentlyAsync.size() == 0 && currentlyAwaiting.size() == 0 ? "done" : "running");
    }

    private boolean executeCommand(String command) {
        // Run command and wait for result
        String[] commandSplit = command.split(" ", 2);
        try {
            Tuple<Boolean, String> result = bot.execute(commandSplit[0], commandSplit[1]);
            // Check if null
            if (result == null || result.getA()) {
                // Return done
                return true;
            }
            return false;
        } catch (Exception ignored) {
            // Return done
            return true;
        }
    }
}
