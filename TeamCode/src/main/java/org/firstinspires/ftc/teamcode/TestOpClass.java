package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class TestOpClass extends LinearOpMode {
    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        waitForStart();

        y *= maxPower;
        x *= maxPower;
        rx *= maxPower;

        while (opModeIsActive()) {
            // Handle drive controls using DriveSubsystem
            robot.driveSubsystem.handleDriveInput(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                gamepad1.left_trigger,
                gamepad1.right_trigger
            );

            // Handle arm controls using ArmSubsystem
            robot.armSubsystem.handleArmMovement(
                gamepad2.dpad_up,
                gamepad2.dpad_down
            );

            // Handle servo controls using ServoSubsystem
            robot.servoSubsystem.handleClawControl(
                gamepad2.a,
                gamepad2.b
            );

            robot.servoSubsystem.handleWristControl(
                gamepad2.x,
                gamepad2.y
            );

            robot.servoSubsystem.handleWristSpinControl(
                gamepad2.left_bumper,
                gamepad2.right_bumper
            );
        }
    }
}