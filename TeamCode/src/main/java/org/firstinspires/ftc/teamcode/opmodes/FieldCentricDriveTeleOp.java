package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU.Parameters;

@TeleOp(name = "Field Centric Drive", group = "TeleOp")
public class FieldCentricDriveTeleOp extends LinearOpMode {
    private DriveSubsystem drive;

    @Override
    public void runOpMode() {
        // Initialize motors
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions (adjust these based on your robot's configuration)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Set parameters for logo forward, USB up orientation
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );
        imu.initialize(parameters);

        // Create drive subsystem
        drive = new DriveSubsystem(frontLeft, backLeft, frontRight, backRight, imu);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Reset IMU heading when the match starts
        drive.calibrateIMU();

        while (opModeIsActive()) {
            // Get gamepad inputs
            double y = -gamepad1.left_stick_y; // Negative because Y axis is reversed on gamepad
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;

            // Toggle field centric mode with B button
            if (gamepad1.b) {
                drive.setFieldCentric(!drive.isFieldCentric());
                while (gamepad1.b) {
                    // Wait for button release to prevent multiple toggles
                }
            }

            // Recalibrate IMU with Y button
            if (gamepad1.y) {
                drive.calibrateIMU();
                telemetry.addData("IMU", "Recalibrated");
                while (gamepad1.y) {
                    // Wait for button release to prevent multiple calibrations
                }
            }

            // Handle drive input
            drive.handleDriveInput(y, x, rx, leftTrigger, rightTrigger);

            // Display telemetry
            telemetry.addData("Field Centric", drive.isFieldCentric() ? "Enabled" : "Disabled");
            telemetry.addData("Robot Heading", "%.1f°", drive.getHeading());
            telemetry.addData("Left Stick", "Y (%.2f), X (%.2f)", y, x);
            telemetry.addData("Right Stick X", "%.2f", rx);
            telemetry.update();
        }
    }
}
