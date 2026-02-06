package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobildasensors.GoBildaPinpointDriver;

public class RobotHardware {
    // Motor names as constants
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";
    public static final String SHOOTER_LEFT_COLOR_SENSOR = "backLeftColorSensor";
    public static final String SHOOTER_RIGHT_COLOR_SENSOR = "backRightColorSensor";
    public static final String FRONT_LEFT_COLOR_SENSOR = "frontLeftColorSensor";
    public static final String FRONT_RIGHT_COLOR_SENSOR = "frontRightColorSensor";
    public static final String BLINKIN_SENSOR = "LEDStrip";
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String FRONT_INTAKE_MOTOR = "frontIntakeMotor";
    public static final String IMU_NAME = "imu";
    public static final String PINPOINT_NAME = "odo"; // I2C Pinpoint odometry computer

    // Subsystems
    public DriveSubsystem driveSubsystem;
    //public OdometrySubsystem odometrySubsystem;
    public ShooterSubsystem shooterSubsystem;
    public IntakeSubsystem intakeSubsystem;
    // Hardware declarations (kept for compatibility)
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor intakeMotor;
    public DcMotor frontIntakeMotor;
    public ElapsedTime timer;
    public IMU imu;
    public GoBildaPinpointDriver pinpoint; // Pinpoint odometry computer
     public ColorSubsystem colorSubsystem = new ColorSubsystem();
    public LimeLight limeLight=new LimeLight();
    private HardwareMap hardwareMap;
    public String shooterSubsystemError = null;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR);
        backLeftMotor = hardwareMap.dcMotor.get(BACK_LEFT_MOTOR);
        frontRightMotor = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        backRightMotor = hardwareMap.dcMotor.get(BACK_RIGHT_MOTOR);
        intakeMotor = hardwareMap.dcMotor.get(INTAKE_MOTOR);
        frontIntakeMotor = hardwareMap.dcMotor.get(FRONT_INTAKE_MOTOR);
        colorSubsystem.init(hardwareMap, SHOOTER_LEFT_COLOR_SENSOR, SHOOTER_RIGHT_COLOR_SENSOR,
                                         FRONT_LEFT_COLOR_SENSOR, FRONT_RIGHT_COLOR_SENSOR, BLINKIN_SENSOR);
        limeLight.init(hardwareMap);
        //Create limelight object

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        timer = new ElapsedTime();

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize Pinpoint odometry computer (I2C)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // Set encoder resolution based on your odometry pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions - adjust if wheels count backwards
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                       GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Configure Pinpoint - set offsets for your dead wheel positions
        // X offset: parallel wheel distance from center (forward/backward) in mm
        // Y offset: perpendicular wheel distance from center (left/right) in mm
        // Measure from robot center to each encoder wheel
        // Positive X = forward, Positive Y = left
        // TODO: MEASURE AND UPDATE THESE VALUES FROM YOUR ROBOT
        pinpoint.setOffsets(-84.0, -168.0); // (xOffset in mm, yOffset in mm)
        pinpoint.resetPosAndIMU();
        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, frontIntakeMotor);

        // Initialize ShooterSubsystem only if hardware is configured
        try {
            shooterSubsystem = new ShooterSubsystem(hardwareMap);
        } catch (Exception e) {
            // ShooterSubsystem hardware not configured, leave it null
            shooterSubsystem = null;
            shooterSubsystemError = "ShooterSubsystem init failed: " + e.getMessage();
        }

        //odometrySubsystem = new OdometrySubsystem(pinpoint);
    }

    // Delegate methods to subsystems
    public void setDrivePowers(double fl, double bl, double fr, double br) {
        driveSubsystem.handleDriveInput(-fl, 0, 0, 0, 0);
    }
}
