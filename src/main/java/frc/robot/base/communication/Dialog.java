package frc.robot.base.communication;

import java.io.*;
import java.net.Socket;

public class Dialog {

    private BufferedReader reader;
    private BufferedWriter writer;

    public Dialog(Socket socket){
        // Setup I/O
        try {
            reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            writer = new BufferedWriter(new OutputStreamWriter(socket.getOutputStream()));
            System.out.println("I/O Setup Completed");
        } catch (IOException e) {
            reader = null;
            writer = null;
            System.out.println("I/O Setup Failed");
        } finally {
            if (reader != null && writer != null) {
                new Thread(() -> {
                    try {
                        // Begin listening
                        while (running) {
                            try {
                                if (reader.ready()) {
                                    input(reader.readLine());
                                }
                                Thread.sleep(10);
                            } catch (IOException e) {
                                System.out.println("Failed to read input stream.");
                            } catch (InterruptedException e) {
                                System.out.println("Failed to sleep.");
                            }
                        }
                    } catch (Exception e) {
                        System.out.println("Unrecoverable exception: " + e.toString());
                    }
                    // Finish listening
                    System.out.println("Finished");
                    try {
                        this.socket.close();
                    } catch (Exception e) {
                        System.out.println("Failed to close socket.");
                    }
                }).start();
            }
    }

}
