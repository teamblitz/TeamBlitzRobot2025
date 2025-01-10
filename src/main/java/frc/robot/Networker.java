package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class Networker {

    private final Notifier notifier;
    private final DatagramSocket socket;
    private final InetAddress address;

    // private volatile boolean running;

    public Networker() throws SocketException, UnknownHostException {
        notifier = new Notifier(this::sendPacket);
        notifier.startPeriodic(Constants.Networking.INTERVAL);

        socket = new DatagramSocket(Constants.Networking.PORT);
        address = InetAddress.getByName(Constants.Networking.JETSON_IP_ADDRESS);
    }

    /** Do NOT Call from the main thread, will induce delays due to io. */
    private void sendPacket() {
        try {
            byte[] buffer =
                    ByteBuffer.allocate(Long.BYTES)
                            .putLong(RobotController.getFPGATime())
                            .order(ByteOrder.BIG_ENDIAN)
                            .array();
            DatagramPacket packet =
                    new DatagramPacket(buffer, buffer.length, address, Constants.Networking.PORT);
            socket.send(packet);
        } catch (IOException e) {
            System.out.println(e);
        } finally {
            stop();
        }
    }

    public void start() {
        notifier.startPeriodic(Constants.Networking.INTERVAL);
    }

    public void stop() {
        notifier.stop();
        socket.close();
    }
}
