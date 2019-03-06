package frc.robot.bobot;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.Timer;
import java.util.TimerTask;

public class TCP extends Subsystem {

    private static final double REFRESH_RATE = 40;
    private static final int PORT = 1234;
    private static final char TERMINATOR = '\0';
    private OutputStream writer;
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private Thread in;

    public TCP() {
        try {
            this.serverSocket = new ServerSocket(PORT);
            this.serverSocket.setSoTimeout((int) getIterationTime());
        } catch (Exception e) {
            log("TCP-Init Error");
        }
    }

    private void waitForClient() {
        try {
            if (serverSocket != null) {
                clientSocket = serverSocket.accept();
                log("Connected");
            }
        } catch (Exception ignored) {
        }
    }

    public void send(final String s) {
        new Thread(() -> {
            try {
                writer = clientSocket.getOutputStream();
                if (!s.isEmpty()) {
                    // Send Output And Flush
                    writer.write((s + TERMINATOR).getBytes());
                }
            } catch (Exception e) {
                if (e instanceof SocketException) disconnected();
            }
        }).start();
    }

    public void listen(OnInput receiver) {
        waitForClient();
        in = new Thread(() -> {
            Timer timer = new Timer();
            timer.scheduleAtFixedRate(new TimerTask() {
                private StringBuilder builder;
                private BufferedReader reader;
                private int character;

                @Override
                public void run() {

                    try {
                        // Init Params
                        if (reader == null)
                            reader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));

                        builder = new StringBuilder();
                        try {
                            // Read Input
                            try {
                                while ((character = reader.read()) != (int) TERMINATOR) {
                                    builder.append((char) character);
                                }
                                //                                reader.close();
                            } catch (Exception e) {
                            }
                            // Parse Input -> Output
                            send(receiver.onInput(builder.toString()));
                        } catch (Exception e) {
                            if (e instanceof SocketException) disconnected();
                        }

                    } catch (Exception ignored) {
                        disconnected();
                    }
                }
            }, 0, getIterationTime());
        });
        in.start();
    }

    public void disconnected() {
        waitForClient();
    }

    private long getIterationTime() {
        return (long) ((double) 1000 / REFRESH_RATE);
    }

    public interface OnInput {
        String onInput(String newInput);
    }
}
