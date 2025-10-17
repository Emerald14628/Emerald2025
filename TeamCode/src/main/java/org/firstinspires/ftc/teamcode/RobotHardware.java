package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotHardware {
    // Motor names as constants
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";
    public static final String ARM_MOTOR = "armMotor";
    public static final String UD_ARM_MOTOR = "udarmMotor";
    public static final String IMU_NAME = "imu";

    // Subsystems
    public DriveSubsystem driveSubsystem;
    //public ArmSubsystem armSubsystem;
    //public ServoSubsystem servoSubsystem;

    // Hardware declarations (kept for compatibility)
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    //public DcMotor armMotor;
    //public DcMotor udarmMotor;
    //public CRServo clawServo;
    //public CRServo wristServo;
    //public CRServo wristSpinServo;
    //public TouchSensor armLimit;
    public ElapsedTime timer;
    public IMU imu;

    private HardwareMap hardwareMap;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR);
        backLeftMotor = hardwareMap.dcMotor.get(BACK_LEFT_MOTOR);
        frontRightMotor = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        backRightMotor = hardwareMap.dcMotor.get(BACK_RIGHT_MOTOR);
        //armMotor = hardwareMap.dcMotor.get(ARM_MOTOR);
        //udarmMotor = hardwareMap.dcMotor.get(UD_ARM_MOTOR);

        // Initialize servos
        //clawServo = hardwareMap.crservo.get("clawServo");
        //wristServo = hardwareMap.crservo.get("wristServo");
        //wristSpinServo = hardwareMap.crservo.get("wristSpinServo");
        //armLimit = hardwareMap.touchSensor.get("armLimit");

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        timer = new ElapsedTime();

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize encoders
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //udarmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);
        //armSubsystem = new ArmSubsystem(armMotor, udarmMotor, armLimit);
        //servoSubsystem = new ServoSubsystem(clawServo, wristServo, wristSpinServo);
    }

    // Delegate methods to subsystems
    public void setDrivePowers(double fl, double bl, double fr, double br) {
        driveSubsystem.handleDriveInput(-fl, 0, 0, 0, 0);
    }

    //public void resetArmEncoders() {armSubsystem.resetEncoders();}
}
