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
        boolean isLeftShooterActive= false;
        boolean isRightShooterActive= false;
        long leftArtifactStartTime = 0;  // Track when left artifact pusher was activated
        long rightArtifactStartTime = 0;  // Track when right artifact pusher was activated
        long hogWheelStartTime = 0;  // Track when hogwheels were activated
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

                //Idea for shooting system
                //1. x button shoots left side
                //   A. start up hogwheels to 100% if not aleady active
                //   B. after x seconds of hogwheel turning on OR shooting articfact (to give time for hogwheels to reach full speed), then activevate left shooter servo
                //   C. after y seconds of left shooter servo active, turn off left shooter servo
                //   D. hogwheels continues to run until x button is pressed again to turn off (Or do we just leave running until the end of the match?)
                //2. b button shoots right side
                //   A. start up hogwheels to 100% if not aleady active
                //   B. after x seconds of hogwheel turning on OR shooting articfact (to give time for hogwheels to reach full speed), then activevate right shooter servo
                //   C. after y seconds of right shooter servo active, turn off right shooter servo
                //   D. hogwheels continues to run until b button is pressed again to turn off (Or do we just leave running until the end of the match?)
                //---------------------------------------
                if ((gamepad1.left_bumper && !lastLeftShooterState) || (gamepad1.right_bumper && !lastRightShooterState)) {
                    // Start hogwheel if not already active
                    if (!isShootingActive) {
                        robot.shooterSubsystem.activateHogWheel();
                        isShootingActive = true;
                        hogWheelStartTime = System.currentTimeMillis();// Record start time
                        if (gamepad1.left_bumper) {
                            isLeftShooterActive = true;
                            leftArtifactStartTime = 0;  // Reset timer
                        } else {
                            isRightShooterActive = true;
                            rightArtifactStartTime = 0; // Reset timer
                        }
                        }
                    else {
                        isLeftShooterActive = false;
                        isRightShooterActive = false;
                        isShootingActive = false;
                        isRightShooterActive=false;
                        isLeftShooterActive=false;
                        isArtifactRightActive=false;
                        isArtifactLeftActive=false;
                        robot.shooterSubsystem.stopShooting();
                        leftArtifactStartTime=0;
                        rightArtifactStartTime=0;
                        hogWheelStartTime =0;

                    }
                }

                    //Shoot left side
                    if (isLeftShooterActive && hogWheelStartTime >0  && System.currentTimeMillis() - hogWheelStartTime >= 2500) {
                        //------------------------
                        //code to use odemetry to move robot to first shooting position goes here
                        //code to adjust shooting angle goes here
                        //------------------------
                        // Shoot left
                            robot.shooterSubsystem.pushArtifactLeft();
                            isArtifactLeftActive = true;
                            leftArtifactStartTime = System.currentTimeMillis();  // Record start time
                        hogWheelStartTime = 0; // Reset hogWheel timer
                    }

                    //Shoot right side
                    if (isRightShooterActive && hogWheelStartTime >0  && System.currentTimeMillis() - hogWheelStartTime >= 2500) {
                        //------------------------
                        //code to use odemetry to move robot to second shooting position goes here
                        //code to adjust shooting angle goes here
                        //------------------------
                        // Shoot right
                            robot.shooterSubsystem.pushArtifactRight();
                            isArtifactRightActive = true;
                            rightArtifactStartTime = System.currentTimeMillis();  // Record start time
                        hogWheelStartTime = 0; // Reset hogWheel timer
                    }


            // Auto-stop left artifact shooter after 3 seconds; will probably need to adjust time later so that
                //it doesnt shoot another artifact if there is one loaded right after
            if (isArtifactLeftActive && leftArtifactStartTime > 0) {
                long elapsedTime = System.currentTimeMillis() - leftArtifactStartTime;
                if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                    robot.shooterSubsystem.stopArtifactLeft();
                    isArtifactLeftActive = false;
                    leftArtifactStartTime = 0;
                }
            }

            // Auto-stop right artifact shooter after 3 seconds; will probably need to adjust time later so that
                //it doesnt shoot another artifact if there is one loaded right after
            if (isArtifactRightActive && rightArtifactStartTime > 0) {
                long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                    robot.shooterSubsystem.stopArtifactRight();
                    isArtifactRightActive = false;
                    rightArtifactStartTime = 0;
                }
            }
                if (gamepad1.b && !lastBButtonState){
                    robot.shooterSubsystem.changeShooterAngleBackward();
                }
                if (!gamepad1.y && lastYButtonState){
                    robot.shooterSubsystem.changeShooterAngleFoward();
                }
/*
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
 //---------------------------------------
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