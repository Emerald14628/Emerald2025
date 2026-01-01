package org.firstinspires.ftc.teamcode;
//comment
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;

@TeleOp
public class TestOpClass extends LinearOpMode {
    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
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
        int opModeLoopCounter = 0;
        boolean lastLeftShooterState= false;
        boolean lastRightShooterState= false;
        boolean isShootingActive= false;
        boolean lastXButtonState= false;
        boolean lastAButtonState= false;
        boolean lastBButtonState= false;
        boolean lastYButtonState= false;
        boolean isIntakeActive= false;
        boolean isArtifactLeftActive= false;
        boolean isArtifactRightActive= false;
        robot.limeLight.start();
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
               ColorSensor.DetectedColor LeftColor= robot.leftColorSensor.getDetectedColor(telemetry);
               ColorSensor.DetectedColor RightColor=robot.rightColorSensor.getDetectedColor(telemetry);
            // Recalibrate IMU with Y button
            if (gamepad1.y) {
                robot.driveSubsystem.calibrateIMU();
                telemetry.addData("IMU", "Recalibrated");
                while (gamepad1.y && opModeIsActive()) {
                    // Wait for button release to prevent multiple calibrations
                }
            }
           // if(opModeLoopCounter == 1500)
            {
               // robot.imu.resetYaw();
                //ModeLoopCounter = 0;
            }
            // Handle drive controls using DriveSubsystem
            robot.driveSubsystem.handleDriveInput(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x, // disable strafing for left joystick
                gamepad1.right_stick_x,
                gamepad1.left_trigger,
                gamepad1.right_trigger);


            // Toggle intake with A button
            if (gamepad1.a && !lastAButtonState) {
                // Button was just pressed - toggle intake state
                if (isIntakeActive) {
                    robot.intakeSubsystem.stopIntake();
                    isIntakeActive = false;
                } else {
                    robot.intakeSubsystem.intakeArtifact();
                    isIntakeActive = true;
                }
            }

            // Shooter controls - only if shooter subsystem initialized successfully
            if (robot.shooterSubsystem != null) {
                if (gamepad1.left_bumper && !lastLeftShooterState) {
                    // Shoot left
                    if (isArtifactLeftActive) {
                        robot.shooterSubsystem.stopArtifactLeft();
                        isArtifactLeftActive = false;
                    } else {
                        robot.shooterSubsystem.pushArtifactLeft();
                        isArtifactLeftActive = true;
                    }
                }
                if (gamepad1.right_bumper && !lastRightShooterState) {
                    // Shoot right
                    if (isArtifactRightActive) {
                        robot.shooterSubsystem.stopArtifactRight();
                        isArtifactRightActive = false;
                    } else {
                        robot.shooterSubsystem.pushArtifactRight();
                        isArtifactRightActive = true;
                    }
                }
                if (gamepad1.x && !lastXButtonState) {
                    if (isShootingActive) {
                        robot.shooterSubsystem.stopHogWheel();
                        isShootingActive = false;
                    } else {
                        robot.shooterSubsystem.activateHogWheel();
                        isShootingActive = true;
                    }
                }
 /*
                    if (gamepad1.b && !lastBButtonState){
                        robot.shooterSubsystem.changeShooterAngleBackward();
                    }
                    if (!gamepad1.y && lastYButtonState){
                        robot.shooterSubsystem.changeShooterAngleFoward();
                    }
                    else {
                        isShootingActive= false;
                        robot.shooterSubsystem.stopShooting();
                    }
    */
            }
            lastLeftShooterState = gamepad1.left_bumper;
            lastRightShooterState = gamepad1.right_bumper;
            lastXButtonState = gamepad1.x;
            lastBButtonState = gamepad1.b;
            lastAButtonState = gamepad1.a;
            lastYButtonState = gamepad1.y;
            telemetry.addData("Version:", "1.0.6");
            telemetry.addData("Description:", "added FCD recalibration");
            robot.driveSubsystem.addMotorPowersToTelemetry(telemetry);
            telemetry.addData("Field Centric", robot.driveSubsystem.isFieldCentric() ? "Enabled" : "Disabled");
            telemetry.addData("Robot Heading", "%.1f°", robot.driveSubsystem.getHeading());

            // Display shooter subsystem error if it failed to initialize
            if (robot.shooterSubsystemError != null) {
                telemetry.addData("⚠ ERROR", robot.shooterSubsystemError);
            }

            telemetry.update();

            opModeLoopCounter ++;
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