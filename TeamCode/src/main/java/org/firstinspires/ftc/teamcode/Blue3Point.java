package org.firstinspires.ftc.teamcode;
//comment

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous
public class Blue3Point extends LinearOpMode {

    enum States{
        TURNLEFTFORTYFIVE, MOVEFORWARD, TURNAROUND, SHOOTLEFT, SHOOTRIGHT,SHOOTRIGHT2ND,TURNRIGHTFORTYFIVE, FINALPOSITION, END
    }
    private RobotHardware robot;
    private States currentState = States.TURNLEFTFORTYFIVE;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        // Variables for button state handling
        boolean lastDpadDownState = false;

        robot.driveSubsystem.calibrateIMU();
        robot.pinpoint.resetPosAndIMU();
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
        long rightArtifactStartTime = 0;  // Track when right artifact pusher was activated
       // robot.limeLight.start();

        robot.pinpoint.initialize();


        if (!isShootingActive) {
            robot.shooterSubsystem.activateHogWheel(.95);
            isShootingActive = true;
            leftArtifactStartTime = 0;  // Res
        }
        while (opModeIsActive()) {

            robot.pinpoint.update();
            switch(currentState)
            {
                case TURNLEFTFORTYFIVE:
                    if(robot.driveSubsystem.getHeading() < 15.0){
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
                case MOVEFORWARD:
                    Pose2D pos = robot.pinpoint.getPosition();
                    // Position is returned in Meters
                    if(Math.sqrt(pos.getX(DistanceUnit.METER)*pos.getX(DistanceUnit.METER)+
                                       pos.getY(DistanceUnit.METER)*pos.getY(DistanceUnit.METER))< 0.3)
                    {
                        robot.driveSubsystem.handleDriveInput(
                                -1.0, 0.0,
                                0.0,
                                0.0,0.0);
                    }
                    else {
                        robot.driveSubsystem.handleDriveInput(
                                0.0, 0.0,
                                0.0,
                                0.0,0.0);
                        currentState = States.TURNAROUND;
                    }
                    break;
                case TURNAROUND:
                    if(robot.driveSubsystem.getHeading() > -100.0){
                        // Handle drive controls using DriveSubsystem
                        robot.driveSubsystem.handleDriveInput(
                                0.0, 0.0,
                                0.6,
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
                           isLeftShooterActive= true;
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
                            //robot.shooterSubsystem.activateHogWheel(0.95);
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
                            isRightShooterActive  = false;
                            rightArtifactStartTime = 0;
                            currentState = States.SHOOTRIGHT2ND;
                            //robot.shooterSubsystem.activateHogWheel(0.95);
                        }
                    }
                    break;
                case SHOOTRIGHT2ND:
                    if (!isRightShooterActive) {
                        robot.shooterSubsystem.pushArtifactRight();
                        robot.intakeSubsystem.intakeArtifact();
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
                            currentState = States.FINALPOSITION;
                            robot.intakeSubsystem.stopIntake();
                        }
                    }
                        break;
                        case TURNRIGHTFORTYFIVE:
                    if(robot.driveSubsystem.getHeading() < -25.0){
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

            telemetry.addData("Version:", "1.0.6");
            telemetry.addData("Description:", "added FCD recalibration");
            robot.driveSubsystem.addMotorPowersToTelemetry(telemetry);
            telemetry.addData("Auto Step", currentState.toString());
            telemetry.addData("Robot Heading", "%.1f°", robot.driveSubsystem.getHeading());

            // Display shooter subsystem error if it failed to initialize
            if (robot.shooterSubsystemError != null) {
                telemetry.addData("⚠ ERROR", robot.shooterSubsystemError);
            }

            telemetry.update();


        }
    }
}