package frc.robot.base.communication;

import java.io.*;
import java.net.Socket;

public class Client {
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
