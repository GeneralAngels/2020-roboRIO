package frc.robot.base.communication;

import frc.robot.base.Bot;

import java.net.ServerSocket;
import java.util.ArrayList;

public class Networking {

    public static final int PORT = 2230;

    private static boolean listening = true;

    private static ServerSocket server = null;
    private static ArrayList<Dialog> dialogs = null;
    private static Bot bot = null;

    static {
        dialogs = new ArrayList<>();
        try {
            server = new ServerSocket(PORT);
            // Listen
            new Thread(() -> {
                while (listening) {
                    if (bot != null) {
                        try {
                            dialogs.add(new Dialog(server.accept()));
                        } catch (Exception e) {
                            System.out.println("Unable to initialize dialog - " + e.getMessage());
                        }
                    } else {
                        try {
                            Thread.sleep(10);
                        } catch (Exception ignored) {
                        }
                    }
                }
            }).start();
        } catch (Exception e) {
            System.out.println("Oh No! - No Comm - " + e.getMessage());
        }
    }

    public static Bot getBot() {
        return bot;
    }

    public static void setBot(Bot bot) {
        Networking.bot = bot;
    }

    public static boolean isListening() {
        return listening;
    }
}
