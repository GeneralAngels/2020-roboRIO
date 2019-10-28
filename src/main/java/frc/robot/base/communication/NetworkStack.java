package frc.robot.base.communication;

import frc.robot.base.Module;

import java.net.DatagramSocket;
import java.net.ServerSocket;
import java.util.Stack;

public class NetworkStack extends Module {

    public static final int PORT = 2230;

    private boolean listen = true;

    private Stack<Packet> outgoing = new Stack<>();
    private Stack<Packet> incoming = new Stack<>();

    private ServerSocket server;

    public NetworkStack() {
        try {
            server = new ServerSocket(PORT);
            // Listen
            new Thread(() -> {
                while(listen){
                    server.
                }
            }).start();
        } catch (Exception e) {
            log("Oh No! - No Comm - " + e.getMessage());
        }
    }
}
