package org.firstinspires.ftc.teamcode;
//comment

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous
public class Red3Point extends LinearOpMode {

    enum States{
        TURNRIGHTFORTYFIVE, MOVEFORWARD, TURNAROUND, SHOOTLEFT, SHOOTRIGHT,SHOOTRIGHT2ND,TURNLEFTTFORTYFIVE, FINALPOSITION, END
    }
    private RobotHardware robot;
    private States currentState = States.TURNRIGHTFORTYFIVE;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        // Variables for button state handling
        boolean lastDpadDownState = false;
        robot.colorSubsystem.Update();
        waitForStart();

        if (isStopRequested()) return;
        // Prepare the sound players
        int enabledSound = hardwareMap.appContext.getResources().getIdentifier("field_centric_enabled", "raw", hardwareMap.appContext.getPackageName());
        int disabledSound = hardwareMap.appContext.getResources().getIdentifier("field_centric_disabled", "raw", hardwareMap.appContext.getPackageName());
        int goteamSound = hardwareMap.appContext.getResources().getIdentifier("go_team_emerald", "raw", hardwareMap.appContext.getPackageName());
        boolean isShootingActive= false;
        boolean isArtifactLeftActive= false;
        boolean isArtifactRightActive= false;
        boolean isLeftShooterActive= false;
        boolean isRightShooterActive= false;
        long leftArtifactStartTime = 0;  // Track when left artifact pusher was activated
        long rightArtifactStartTime = 0;
        long activateHogwheelStartTime = 0;        ;  // Track when right artifact pusher was activated
       // robot.limeLight.start();
        robot.driveSubsystem.calibrateIMU();
        robot.pinpoint.initialize();
        robot.pinpoint.resetPosAndIMU();
        if (!isShootingActive) {
            robot.shooterSubsystem.activateHogWheel(ShooterSubsystem.HogWheelPower.POWER_3);
            isShootingActive = true;
            leftArtifactStartTime = 0;  // Res
        }
            while (opModeIsActive()) {

                robot.pinpoint.update();
                switch (currentState) {
                    case TURNRIGHTFORTYFIVE:
                        if (robot.driveSubsystem.getHeading() > -8.0) {
                            // Handle drive controls using DriveSubsystem
                            robot.driveSubsystem.handleDriveInput(
                                    0.0, 0.0,
                                    0.6,
                                    0.0, 0.0);
                        } else {
                            robot.driveSubsystem.handleDriveInput(
                                    0.0, 0.0,
                                    0.0,
                                    0.0, 0.0);
                            currentState = States.MOVEFORWARD;
                            activateHogwheelStartTime = System.currentTimeMillis();
                        }
                        break;
                    case MOVEFORWARD:
                        long diffrence = System.currentTimeMillis() - activateHogwheelStartTime;
                        if (diffrence > 2500) {
                            currentState = States.SHOOTLEFT;
                }

                    break;
                case TURNAROUND:
                    if(robot.driveSubsystem.getHeading() < 100.0){
                        // Handle drive controls using DriveSubsystem
                        robot.driveSubsystem.handleDriveInput(
                                0.0, 0.0,
                                -0.6,
                                0.0,0.0);
                    }
                    else {
                        robot.driveSubsystem.handleDriveInput(
                                0.0, 0.0,
                                0.0,
                                0.0,0.0);
                        currentState = States.SHOOTLEFT;
                    }
                    break;

                case SHOOTLEFT:
                    if (!isLeftShooterActive) {
                            //------------------------
                            //code to use odemetry to move robot to first shooting position goes here
                            //code to adjust shooting angle goes here
                            //------------------------
                            // Shoot left
                            robot.shooterSubsystem.pushArtifactLeft();
                            isLeftShooterActive = true;
                            isArtifactLeftActive = true;
                            leftArtifactStartTime = System.currentTimeMillis();  // Record start time
                        }
                    else  if (isArtifactLeftActive && leftArtifactStartTime > 0) {
                        long elapsedTime = System.currentTimeMillis() - leftArtifactStartTime;
                        if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                            robot.shooterSubsystem.stopArtifactLeft();
                            isArtifactLeftActive = false;
                            leftArtifactStartTime = 0;
                            currentState = States.SHOOTRIGHT;
                            //robot.shooterSubsystem.activateHogWheel(.95);
                        }
                    }
                    break;
                case SHOOTRIGHT:
                    if (!isRightShooterActive) {
                        //code to use odemetry to move robot to first shooting position goes here
                        //code to adjust shooting angle goes here
                        //------------------------
                        // Shoot left
                        robot.shooterSubsystem.pushArtifactRight();
                        isArtifactRightActive = true;
                        isRightShooterActive = true;
                        rightArtifactStartTime = System.currentTimeMillis();  // Record start time
                    }
                    else  if (isArtifactRightActive && rightArtifactStartTime > 0) {
                        long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                        if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                            isArtifactRightActive = false;
                            rightArtifactStartTime = 0;
                            isRightShooterActive = false;
                            currentState = States.SHOOTRIGHT2ND;
                        }
                    }
                    break;
                case SHOOTRIGHT2ND:
                    if (!isRightShooterActive) {
                        robot.shooterSubsystem.pushArtifactRight();
                        robot.intakeSubsystem.intakeArtifactStage2();
                        isArtifactRightActive = true;
                        isRightShooterActive = true;
                        rightArtifactStartTime = System.currentTimeMillis();  // Record start time
                    }
                    else  if (isArtifactRightActive && rightArtifactStartTime > 0) {
                        long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                        if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                            isArtifactRightActive = false;
                            rightArtifactStartTime = 0;
                            robot.shooterSubsystem.stopShooting();
                            currentState = States.  FINALPOSITION;
                            robot.intakeSubsystem.stopIntake();
                        }
                    }
                        break;
                case TURNLEFTTFORTYFIVE:
                    if(robot.driveSubsystem.getHeading() > 25.0){
                        // Handle drive controls using DriveSubsystem
                        robot.driveSubsystem.handleDriveInput(
                                0.0, 0.0,
                                -0.6,
                                0.0,0.0);
                    }
                    else {
                        robot.driveSubsystem.handleDriveInput(
                                0.0, 0.0,
                                0.0,
                                0.0,0.0);
                        currentState = States.FINALPOSITION;
                    }
                    break;
                case FINALPOSITION:
                    Pose2D finalPos = robot.pinpoint.getPosition();
                    // Position is returned in Meters
                    if(Math.sqrt(finalPos.getX(DistanceUnit.METER)*finalPos.getX(DistanceUnit.METER)+
                            finalPos.getY(DistanceUnit.METER)*finalPos.getY(DistanceUnit.METER))< 0.6)
                    {
                        robot.driveSubsystem.handleDriveInput(
                                -0.4, 0.0,
                                0.0,
                                0.0,0.0);
                    }
                    else {
                        robot.driveSubsystem.handleDriveInput(
                                0.0, 0.0,
                                0.0,
                                0.0,0.0);
                        currentState = States.END;
                    }
                    break;
                case END:
                    break;
            }

            telemetry.addData("Version:", "1.0.7");
            telemetry.addData("Description:", "corrected red auto turn after shooting");
            robot.driveSubsystem.addMotorPowersToTelemetry(telemetry);
            telemetry.addData("Auto Step", currentState.toString());
            telemetry.addData("Robot Heading", "%.1f°", robot.driveSubsystem.getHeading());

            // Display shooter subsystem error if it failed to initialize
            if (robot.shooterSubsystemError != null) {
                telemetry.addData("⚠ ERROR", robot.shooterSubsystemError);
            }

            telemetry.update();
                robot.colorSubsystem.Update();

        }
    }
}