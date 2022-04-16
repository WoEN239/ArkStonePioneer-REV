package org.firstinspires.ftc.teamcode.usbserial;

import static org.firstinspires.ftc.teamcode.usbserial.ArrayUtils.concatArray;
import static org.firstinspires.ftc.teamcode.usbserial.ArrayUtils.truncateArray;

import android.app.Activity;
import android.content.Context;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class SerialConnection {

    private static final int BAUD_RATE = 115200;
    private static final int USB_VID = 0x0403;
    private static final int USB_PID = 0x6001;
    private static final boolean FILTER_ID = false;
    private static final int TIMEOUT_MILLIS = 1000;
    private static final int RECEIVE_MESSAGE_LENGTH = 4;
    private static final int BUFFER_SIZE = 128;

    private final UsbManager usbManager;
    private final UsbSerialProber usbSerialProber;
    private ScheduledExecutorService commandSenderExecutorService = null;
    private UsbSerialPort usbSerialPort = null;
    private static boolean userOpModeRunning = false;
    private boolean connectionIsActive = false;

    public SerialConnection() {
        this(AppUtil.getInstance().getRootActivity());
    }

    public SerialConnection(Activity activity) {
        usbManager = (UsbManager) activity.getSystemService(Context.USB_SERVICE);
        OpModeManagerNotifier.Notifications stopOpModeNotifier = new OpModeManagerNotifier.Notifications() {
            @Override
            public void onOpModePreInit(OpMode opMode) {
                userOpModeRunning = !(opMode instanceof OpModeManagerImpl.DefaultOpMode);
            }

            @Override
            public void onOpModePreStart(OpMode opMode) {
            }

            @Override
            public void onOpModePostStop(OpMode opMode) {
                userOpModeRunning = false;
                try {
                    close();
                } catch (IOException ignored) {
                }
            }
        };
        userOpModeRunning = true;
        OpModeManagerImpl.getOpModeManagerOfActivity(activity).registerListener(stopOpModeNotifier);
        ProbeTable probeTable = new ProbeTable().addProduct(USB_VID, USB_PID, CdcAcmSerialDriver.class);
        usbSerialProber = FILTER_ID ? new UsbSerialProber(probeTable) : UsbSerialProber.getDefaultProber();
    }

    public boolean isConnectionActive() {
        return connectionIsActive;
    }

    public byte[] sendCommand(SerialCommand serialCommand) {
        try {
            return commandSenderExecutorService.submit(() -> writeBytesAndGetResponse(serialCommand.getByteArray(), RECEIVE_MESSAGE_LENGTH)).get(TIMEOUT_MILLIS, TimeUnit.MILLISECONDS);
        } catch (ExecutionException | InterruptedException | TimeoutException e) {
            return null;
        }
    }

    private void writeBytes(byte[] bytesToWrite) throws IOException {
        writeBytesAndGetResponse(bytesToWrite, 0);
    }

    private synchronized byte[] writeBytesAndGetResponse(byte[] bytesToWrite, int responseLength) throws IOException {
        try {
            usbSerialPort.write(bytesToWrite, TIMEOUT_MILLIS);
            try {
                byte[] receivedMessage = new byte[0];
                ElapsedTime receiveTimer = new ElapsedTime();
                while (userOpModeRunning && receiveTimer.milliseconds() < TIMEOUT_MILLIS) {
                    byte[] buffer = new byte[BUFFER_SIZE];
                    int read = usbSerialPort.read(buffer, TIMEOUT_MILLIS);
                    if (read > 0)
                        receivedMessage = concatArray(receivedMessage, truncateArray(buffer, read));
                    if (receivedMessage.length >= responseLength)
                        return truncateArray(receivedMessage, responseLength);
                }
                throw new IOException("Timeout exceeded");
            } catch (IOException exception) {
                throw new IOException("Read failed (" + exception.getMessage() + ")");
            }
        } catch (IOException exception) {
            throw new IOException("Write failed (" + exception.getMessage() + ")");
        }
    }

    public synchronized void open() throws IOException {

        try {
            close();
        } catch (IOException exception) {
        }

        UsbSerialDriver driver;

        // Open a connection to the first available driver.
        try {
            driver = usbSerialProber.findAllDrivers(usbManager).get(0);
        } catch (IndexOutOfBoundsException e) {
            throw new IOException("No available driver found");
        }

        UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
        if (connection == null) {
            throw new IOException("Driver found, can't open device");
        }

        usbSerialPort = driver.getPorts().get(0); // Most devices have just one port (port 0)

        usbSerialPort.open(connection);
        usbSerialPort.setParameters(BAUD_RATE, UsbSerialPort.DATABITS_8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
        if (commandSenderExecutorService != null)
            commandSenderExecutorService.shutdownNow();
        commandSenderExecutorService = Executors.newScheduledThreadPool(2);
        commandSenderExecutorService.scheduleWithFixedDelay(() -> {
            try {
                if (userOpModeRunning)
                    writeBytesAndGetResponse(new SerialCommand(SerialCommand.CommandType.CMD_SETMOTORPOWER, 0, 1, -50).getByteArray(), 4);
                else
                    writeBytesAndGetResponse(new SerialCommand(SerialCommand.CommandType.CMD_HEARTBEAT).getByteArray(), 4);
            } catch (IOException exception) {
                connectionIsActive = false;
                try {
                    open();
                } catch (IOException ignored) {
                }
            }
        }, TIMEOUT_MILLIS / 2, TIMEOUT_MILLIS / 2, TimeUnit.MILLISECONDS);
        connectionIsActive = true;
    }

    public synchronized void close() throws IOException {
        connectionIsActive = false;
        if (commandSenderExecutorService != null)
            commandSenderExecutorService.shutdownNow();
        if (usbSerialPort != null)
            usbSerialPort.close();
    }


}
