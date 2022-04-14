package org.firstinspires.ftc.teamcode.usbserial;


import android.content.Context;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;
import java.math.BigInteger;
import java.util.Arrays;
import java.util.List;

@TeleOp
@Config
public class Test extends LinearOpMode {

    /*
    public static List<byte[]> tokens(byte[] array, byte[] delimiter) {
        List<byte[]> byteArrays = new LinkedList<>();
        if (delimiter.length == 0) {
            return byteArrays;
        }
        int begin = 0;

        outer:
        for (int i = 0; i < array.length - delimiter.length + 1; i++) {
            for (int j = 0; j < delimiter.length; j++) {
                if (array[i + j] != delimiter[j]) {
                    continue outer;
                }
            }
            byteArrays.add(Arrays.copyOfRange(array, begin, i));
            begin = i + delimiter.length;
        }
        byteArrays.add(Arrays.copyOfRange(array, begin, array.length));
        return byteArrays;
    }
     */

    private static final int BAUD_RATE = 115200;
    private static final int USB_VID = 0x0403;
    private static final int USB_PID = 0x6001;
    private static final boolean FILTER_ID = false;
    private static final int TIMEOUT_MILLIS = 1000;
    private static final int RECEIVE_MESSAGE_LENGTH = 4;
    private static final int BUFFER_SIZE = 128;

    public static CommandType commandType = CommandType.CMD_RESET;
    public static int controllerNumber = 0;
    public static int hiByte = 0;
    public static int loByte = 0;

    public static int motorNumber = 1;

    static byte[] concatArray(byte[] array1, byte[] array2) {
        byte[] result = Arrays.copyOf(array1, array1.length + array2.length);
        System.arraycopy(array2, 0, result, array1.length, array2.length);
        return result;
    }

    static byte[] truncateArray(byte[] array, int newLength) {
        if (array.length < newLength) {
            return array;
        } else {
            byte[] truncated = new byte[newLength];
            System.arraycopy(array, 0, truncated, 0, newLength);
            return truncated;
        }
    }

    public UsbSerialPort establishConnection() throws IOException {
        UsbManager manager = (UsbManager) hardwareMap.appContext.getSystemService(Context.USB_SERVICE);

        ProbeTable customTable = new ProbeTable().addProduct(USB_VID, USB_PID, CdcAcmSerialDriver.class);
        List<UsbSerialDriver> availableDrivers =
                (FILTER_ID ? new UsbSerialProber(customTable) : UsbSerialProber.getDefaultProber()).findAllDrivers(manager);

        if (availableDrivers.isEmpty()) {
            throw new IOException("No available driver found");
        }

        // Open a connection to the first available driver.
        UsbSerialDriver driver = availableDrivers.get(0);
        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
        if (connection == null) {
            throw new IOException("Driver found, can't establish connection");
        }

        UsbSerialPort port = driver.getPorts().get(0); // Most devices have just one port (port 0)

        port.open(connection);
        port.setParameters(BAUD_RATE, UsbSerialPort.DATABITS_8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
        return port;
    }

    public byte[] writeBytesAndGetResponse(UsbSerialPort serialPort, byte[] bytesToWrite) throws IOException {
        try {
            serialPort.write(bytesToWrite, TIMEOUT_MILLIS);
            try {
                byte[] receivedMessage = new byte[0];
                ElapsedTime receiveTimer = new ElapsedTime();
                receiveTimer.reset();
                while (opModeIsActive() && receiveTimer.milliseconds() < TIMEOUT_MILLIS) {
                    byte[] buffer = new byte[BUFFER_SIZE];
                    int read = serialPort.read(buffer, TIMEOUT_MILLIS);
                    if (read > 0) {
                        receivedMessage = concatArray(receivedMessage, truncateArray(buffer, read));
                    }
                    if (receivedMessage.length >= RECEIVE_MESSAGE_LENGTH) {
                        return truncateArray(receivedMessage, RECEIVE_MESSAGE_LENGTH);
                    }
                }
                throw new IOException("Timeout exceeded");
            } catch (IOException exception) {
                throw new IOException("Read failed (" + exception.getMessage() + ")");
            }
        } catch (IOException exception) {
            throw new IOException("Write failed (" + exception.getMessage() + ")");
        }
    }

    public byte[] writeCommand(UsbSerialPort serialPort, CommandType commandType, int controllerNumber, int hiByte, int loByte) throws IOException {
        return writeBytesAndGetResponse(serialPort, new byte[]{(byte) commandType.ordinal(), (byte) controllerNumber, (byte) hiByte, (byte) loByte});
    }

    @Override
    public void runOpMode() {


        final int WRITE_WAIT_MILLIS = 1000;


        //  telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.setMsTransmissionInterval(1000 / 50);

        UsbSerialPort serialPort = null;

        try {
            serialPort = establishConnection();
        } catch (IOException exception) {
            telemetry.addData("Failed to connect (", exception.getMessage() + ")");
        }
        if (serialPort != null) {
            telemetry.addLine("Connected");
        }

        telemetry.update();
        ElapsedTime elapsedTime = new ElapsedTime();
        waitForStart();

        while (opModeIsActive() && serialPort == null) {
            telemetry.addLine("Trying to re-establish connection");
            try {
                serialPort = establishConnection();
            } catch (IOException exception) {
                telemetry.addData("Connection IOException", exception.getMessage());
            }
            telemetry.update();
        }
        if (serialPort != null) {
            while (opModeIsActive()) {
                try {
                    writeCommand(serialPort, CommandType.CMD_SETMOTORPOWER, 0, motorNumber, (int) (100.0 * gamepad1.left_stick_y));
                    byte[] response = writeCommand(serialPort, commandType, 0, motorNumber, 0);
                    switch (commandType) {
                        case CMD_READENCODER:
                            FtcDashboard.getInstance().getTelemetry().addData("ticks", new BigInteger(response).intValue());
                            FtcDashboard.getInstance().getTelemetry().update();
                        case CMD_READMOTORCURRENT:
                            FtcDashboard.getInstance().getTelemetry().addData("current", new BigInteger(response).intValue());
                            FtcDashboard.getInstance().getTelemetry().update();
                            //break;
                        default:
                            telemetry.addData("Received", Arrays.toString(response));
                            break;
                    }
                } catch (IOException exception) {
                    telemetry.addData("IOException", exception.getMessage());
                    try {
                        serialPort = establishConnection();
                    } catch (IOException fatalException) {
                        telemetry.addData("Failed to reconnect", fatalException.getMessage());
                    }
                }


                sleep(1000 / 50);

                telemetry.update();
            }
            try {
                serialPort.close();
            } catch (IOException ignored) {
            }
        }
    }
}
