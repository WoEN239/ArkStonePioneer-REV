package org.firstinspires.ftc.teamcode.usbserial;


//@TeleOp
//@Config
public class SerialTest /*extends LinearOpMode */ {

    /*
    private static final int BAUD_RATE = 115200;
    private static final int USB_VID = 0x0403;
    private static final int USB_PID = 0x6001;
    private static final boolean FILTER_ID = false;
    private static final int TIMEOUT_MILLIS = 1000;
    private static final int RECEIVE_MESSAGE_LENGTH = 4;
    private static final int BUFFER_SIZE = 128;

    public static SerialCommand.CommandType commandType = SerialCommand.CommandType.CMD_HEARTBEAT;
    public static int controllerNumber = 0;
    public static int hiByte = 0;
    public static int loByte = 0;

    public static int motorNumber = 1;

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

    public byte[] writeCommand(UsbSerialPort serialPort, SerialCommand.CommandType commandType, int controllerNumber, int hiByte, int loByte) throws IOException {
        return writeBytesAndGetResponse(serialPort, new byte[]{(byte) commandType.ordinal(), (byte) controllerNumber, (byte) hiByte, (byte) loByte});
    }


     */

    /*

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        DcMotor dcMotor = hardwareMap.dcMotor.get("motor1");

        dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.setMsTransmissionInterval(1000 / 50);

        sleep(1000 / 50);

        SerialConnection serialConnection = new SerialConnection();

        try {
            serialConnection.open();
        } catch (IOException exception) {
            telemetry.addData("Failed to connect", exception.getMessage());
        }

        telemetry.addData("Connected", serialConnection.isConnectionActive());

        telemetry.update();
        ElapsedTime elapsedTime = new ElapsedTime();
        waitForStart();

        while (opModeIsActive() && !serialConnection.isConnectionActive()) {
            telemetry.addLine("Trying to re-establish connection");
            try {
                serialConnection.open();
            } catch (IOException exception) {
                telemetry.addData("Connection IOException", exception.getMessage());
            }
            telemetry.update();
        }
        elapsedTime.reset();
        while (opModeIsActive()) {
            //if (serialConnection.isConnectionActive()) {
            double power = .25 * sin(elapsedTime.seconds() * 5); //* sin(elapsedTime.seconds() * 3.14);
            serialConnection.sendCommand(new SerialCommand(SerialCommand.CommandType.CMD_SETMOTORPOWER, 0, 1, (int) (100.0 * power)));
            serialConnection.sendCommand(new SerialCommand(SerialCommand.CommandType.CMD_SETMOTORPOWER, 0, 2, (int) ((gamepad1.left_trigger - gamepad1.right_trigger) * 100.0)));
            dcMotor.setPower(power);
            byte[] response = serialConnection.sendCommand(new SerialCommand(SerialCommand.CommandType.CMD_READENCODER, 0, 1));
            if (response != null) {
                //telemetry.addData("Received", Arrays.toString(response));
                telemetry.addData("Value", ArrayUtils.getIntFromBytes(response));
            } else {
                //telemetry.addLine("Received null");
            }
            telemetry.update();
        }
    }

     */
}
