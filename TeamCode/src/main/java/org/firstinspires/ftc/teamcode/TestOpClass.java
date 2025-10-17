package org.firstinspires.ftc.teamcode;
//comment
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftccommon.SoundPlayer;

@TeleOp
public class TestOpClass extends LinearOpMode {
    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //test intake motor
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        robot = new RobotHardware(hardwareMap);
        robot.init();

        // Variables for button state handling
        boolean lastDpadDownState = false;

        waitForStart();

        if (isStopRequested()) return;
        // Prepare the sound players
        int enabledSound = hardwareMap.appContext.getResources().getIdentifier("field_centric_enabled", "raw", hardwareMap.appContext.getPackageName());
        int disabledSound = hardwareMap.appContext.getResources().getIdentifier("field_centric_disabled", "raw", hardwareMap.appContext.getPackageName());
        int goteamSound = hardwareMap.appContext.getResources().getIdentifier("go_team_emerald", "raw", hardwareMap.appContext.getPackageName());

        while (opModeIsActive()) {
            // Handle field centric toggle with dpad_down
            boolean currentDpadDownState = gamepad1.dpad_down;
            if (currentDpadDownState && !lastDpadDownState) {
                // Toggle field centric mode
                boolean newMode = !robot.driveSubsystem.isFieldCentric();
                robot.driveSubsystem.setFieldCentric(newMode);
                // Play the appropriate sound for the new mode
                int soundToPlay = newMode ? enabledSound : disabledSound;
                if (soundToPlay != 0) {
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundToPlay);
                }
            }
            lastDpadDownState = currentDpadDownState;

            // Recalibrate IMU with Y button
            if (gamepad1.y) {
                robot.driveSubsystem.calibrateIMU();
                telemetry.addData("IMU", "Recalibrated");
                while (gamepad1.y && opModeIsActive()) {
                    // Wait for button release to prevent multiple calibrations
                }
            }

            // Handle drive controls using DriveSubsystem
            robot.driveSubsystem.handleDriveInput(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                gamepad1.left_trigger,
                gamepad1.right_trigger);

            //test intake dc motor
            double intakeSpeed=0;
            if (gamepad1.triangle) {
                intakeSpeed = -1.0;
            }
            intakeMotor.setPower(intakeSpeed);

            if (gamepad1.a) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goteamSound);
            }

            telemetry.addData("Version:", "1.0.6");
            telemetry.addData("Description:", "added FCD recalibration");
            robot.driveSubsystem.addMotorPowersToTelemetry(telemetry);
            telemetry.addData("Field Centric", robot.driveSubsystem.isFieldCentric() ? "Enabled" : "Disabled");
            telemetry.addData("Robot Heading", "%.1f°", robot.driveSubsystem.getHeading());
            telemetry.update();

            // Handle arm controls using ArmSubsystem
            //robot.armSubsystem.handleArmMovement(
            //    gamepad2.dpad_up,
            //    gamepad2.dpad_down
            //);

            // Handle servo controls using ServoSubsystem
            //robot.servoSubsystem.handleClawControl(
            //    gamepad2.a,
            //    gamepad2.b
            //);

            //robot.servoSubsystem.handleWristControl(
            //    gamepad2.x,
            //    gamepad2.y
            //);

            //robot.servoSubsystem.handleWristSpinControl(
            //    gamepad2.left_bumper,
            //    gamepad2.right_bumper
            //);
        }
    }
}