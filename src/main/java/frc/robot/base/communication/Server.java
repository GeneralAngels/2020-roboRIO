package frc.robot.base.communication;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;
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

    private static class Client {
        private boolean running = true;
        private BufferedReader reader;
        private BufferedWriter writer;

        public Client(Socket socket, Node master) {
            // Setup I/O
            try {
                reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                writer = new BufferedWriter(new OutputStreamWriter(socket.getOutputStream()));
            } catch (IOException e) {
                reader = null;
                writer = null;
                System.out.println("Unable to connect to client");
            } finally {
                if (reader != null && writer != null) {
                    new Thread(() -> {
                        try {
                            // Begin listening
                            while (running) {
                                if (reader.ready()) {
                                    String received = reader.readLine();
                                    Node.Tuple<Boolean, String> output = null;
                                    try {
                                        String[] split = received.split(" ", 3);
                                        if (split.length >= 2) {
                                            Node node = master.find(split[0]);
                                            if (node != null) {
                                                if (split.length >= 3)
                                                    output = node.execute(split[1], split[2]);
                                                else
                                                    output = node.execute(split[1], null);
                                            }
                                        }
                                    } catch (Exception ignored) {
                                    }
                                    if (output != null) {
                                        writer.write(output.getB());
                                    }
                                    writer.newLine();
                                    writer.flush();
                                }
                                Thread.sleep(10);
                            }
                        } catch (Exception e) {
                            System.out.println("Unrecoverable exception: " + e.toString());
                        }

                        try {
                            socket.close();
                        } catch (Exception e) {
                            System.out.println("Unrecoverable exception: " + e.toString());
                        }

                    }).start();
                }
            }
        }
    }
}
