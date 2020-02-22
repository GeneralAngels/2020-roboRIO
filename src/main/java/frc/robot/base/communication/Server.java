package frc.robot.base.communication;

import java.net.ServerSocket;
import java.util.ArrayList;

import static java.lang.Thread.sleep;

public class Server {

    public static final int PORT = 5800;

    private static boolean listening = true;
    private static ServerSocket server = null;
    private static ArrayList<Client> clients = null;

    public static void begin(Node master) {
        clients = new ArrayList<>();
        try {
            server = new ServerSocket(PORT);
            // Listen
            new Thread(() -> {
                while (listening) {
                    try {
                        clients.add(new Client(server.accept(), master));
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                    try {
                        sleep(1000);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }).start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
