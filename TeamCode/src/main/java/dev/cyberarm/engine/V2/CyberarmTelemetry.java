package dev.cyberarm.engine.V2;

import android.util.Log;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Locale;

/**
 * One way (Robot Controller -> Client), UDP based, telemetry via multicast.
 */
public class CyberarmTelemetry {
    private static final int PROTOCOL_IDENTIFIER = 0x54494d45;
    private static final int MAX_PACKET_RAW_SIZE = 508; // bytes
    private static final int PACKET_HEADER_SIZE = 16; // bytes
    private static final int MAX_PACKET_BODY_SIZE = MAX_PACKET_RAW_SIZE - PACKET_HEADER_SIZE; // bytes
    private static final String MULTICAST_ADDRESS = "236.77.83.76";
    private static final int MULTICAST_PORT = 6388;
    private static final int MULTICAST_TTL = 5;
    private static final String TAG = "CYBERARM_TELEMETRY";
    private final ArrayList<ByteArrayOutputStream> queueBuffer = new ArrayList<>();
    private InetAddress group;
    private MulticastSocket multicastSocket;

    private boolean usable = false;

    enum Encode {
        // Generic/Type encodings
        INTEGER,
        LONG,
        FLOAT,
        DOUBLE,
        STRING,
        BOOLEAN,

        // Special encodings
        POSE_POSITION, // position of robot in 2D space, relative to field origin
        TELEMETRY, // string telemetry
        GAMEPAD, // all 15 buttons + joysticks + triggers input values
        MOTOR, // current power, velocity, position, target position, and current (amps)
        SERVO, // current target position
        CONTINUOUS_SERVO, // current power
        SENSOR_2M_DISTANCE, // Rev 2 meter distance sensor
        SENSOR_DIGITAL, // touch or other digital/binary sensor
    }

    CyberarmTelemetry() {
        try {
            this.group = InetAddress.getByName(MULTICAST_ADDRESS);
            this.multicastSocket = new MulticastSocket(MULTICAST_PORT);
            this.multicastSocket.setTimeToLive(MULTICAST_TTL);
            this.multicastSocket.joinGroup(group);

            this.usable = true;
        } catch (IOException e) {
            Log.e(TAG, "FAILED to create multicast socket!");
            e.printStackTrace();

            this.usable = false;
        }
    }

    /**
     * Whether the multicast socket was successfully created
     * @return usable
     */
    public boolean isUsable() {
        return usable;
    }

    public void publish() throws IOException
    {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();

        for (ByteArrayOutputStream data : queueBuffer) {
            if (buffer.size() + data.size() >= MAX_PACKET_BODY_SIZE) {
                ByteArrayOutputStream output = new ByteArrayOutputStream();

                packHeader(output, buffer);
                commitPacket(output);

                buffer.reset();
            }

            buffer.write(data.toByteArray());
        }

        // We're a lossy protocol, assume that all data in the buffer has been processed and committed
        queueBuffer.clear();
    }

    public void addPose(double x, double y, double r) {
        addPosition(x, y, r);
    }

    public void addPosition(double x, double y, double r) {
        try {
            ByteArrayOutputStream buffer = new ByteArrayOutputStream();

            buffer.write(Encode.POSE_POSITION.ordinal());
            packDouble(buffer, x);
            packDouble(buffer, y);
            packDouble(buffer, r);

            queueBuffer.add(buffer);
        } catch (IOException e) {
            Log.e(TAG, "An error occurred while adding robot Pose (Position) to queue");
            e.printStackTrace();
        }
    }

    public void addTelemetry(String msg) {
        try {
            ByteArrayOutputStream buffer = new ByteArrayOutputStream();

            buffer.write(Encode.TELEMETRY.ordinal());
            packString(buffer, msg);

            queueBuffer.add(buffer);
        } catch (IOException e) {
            Log.e(TAG, "An error occurred while adding Telemetry message to queue");
            e.printStackTrace();
        }
    }

    // ------- HELPERS -------- //
    private void packHeader(ByteArrayOutputStream output, ByteArrayOutputStream buffer) throws IOException {
        output.write(PROTOCOL_IDENTIFIER);
        output.write(buffer.size()); // BUFFER SIZE (multiple buffers may fit in one packet)
        output.write(buffer.toByteArray());
    }

    private void commitPacket(ByteArrayOutputStream buffer) throws IOException {
        // TODO: send multicast packet(s) on LAN

        // Drop packet if multicast socket is not yet created or failed to be created.
        if (!isUsable())
            return;

        DatagramPacket packet = new DatagramPacket(buffer.toByteArray(), buffer.size(), group, MULTICAST_PORT);
        multicastSocket.send(packet);
    }

    private void packInt(ByteArrayOutputStream stream, int i) throws IOException {
        stream.write(Encode.INTEGER.ordinal());
        stream.write(i);
    }

    private void packLong(ByteArrayOutputStream stream, long i) throws IOException {
        stream.write(Encode.LONG.ordinal());
        stream.write(Long.toBinaryString(i).getBytes());
    }

    private void packFloat(ByteArrayOutputStream stream, float i) throws IOException {
        stream.write(Encode.FLOAT.ordinal());
        String[] string = String.format(Locale.US, "%.8f", i).split("\\.");
        int wholeNum = Integer.parseInt(string[0]);
        int decimalPart = Integer.parseInt(string[1]);

        stream.write(wholeNum);
        stream.write(decimalPart);
    }

    private void packDouble(ByteArrayOutputStream stream, double i) throws IOException {
        stream.write(Encode.DOUBLE.ordinal());
        String[] string = String.format(Locale.US, "%.8f", i).split("\\.");
        int wholeNum = Integer.parseInt(string[0]);
        int decimalPart = Integer.parseInt(string[1]);

        stream.write(wholeNum);
        stream.write(decimalPart);
    }

    private void packString(ByteArrayOutputStream stream, String i) throws IOException {
        stream.write(Encode.STRING.ordinal());
        stream.write(i.length());
        stream.write(i.getBytes(StandardCharsets.UTF_8));
    }

    private void packBoolean(ByteArrayOutputStream stream, boolean i) throws IOException {
        stream.write(Encode.BOOLEAN.ordinal());
        byte[] bit = {(byte) (i ? 1 : 0)};
        stream.write(bit);
    }
}
