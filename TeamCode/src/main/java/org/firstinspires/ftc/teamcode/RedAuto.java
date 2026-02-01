package org.firstinspires.ftc.teamcode;
//comment

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.TargetPosition;

@Autonomous
public class RedAuto extends LinearOpMode {

    enum States{
        TURNTOLAUNCHZONE, MOVEFORWARD, AIMATTARGET, SHOOTGPP, SHOOTPGP, SHOOTPPG,TURNTOFINALPOSITION, FINALPOSITION, END
    }
    private RobotHardware robot;
    private States currentState = States.TURNTOLAUNCHZONE;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        // Variables for button state handling
        boolean lastDpadDownState = false;
        LimeLight.Motif currentMotif = LimeLight.Motif.UNKNOWN;
        robot.colorSubsystem.Update();
        robot.limeLight.start();
        waitForStart();

        if (isStopRequested()) return;
        // Prepare the sound players
        boolean isShootingActive= false;
        double aimRx = 0.0;

        robot.driveSubsystem.calibrateIMU();
        robot.pinpoint.initialize();
        robot.pinpoint.resetPosAndIMU();
        if (!isShootingActive) {
            robot.shooterSubsystem.activateHogWheel(.65);
            isShootingActive = true;
        }
            while (opModeIsActive()) {

            robot.pinpoint.update();
            if(currentMotif == LimeLight.Motif.UNKNOWN)
            {
                currentMotif = robot.limeLight.readMotif(robot.imu.getRobotYawPitchRollAngles().getYaw());
            }
            switch(currentState)
            {
                case TURNTOLAUNCHZONE:
                    if(robot.driveSubsystem.getHeading () > -45.0){
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
                        currentState = States.MOVEFORWARD;
                    }
                    break;
                case MOVEFORWARD:
                    Pose2D pos = robot.pinpoint.getPosition();
                    // Position is returned in Meters
                    if(Math.sqrt(pos.getX(DistanceUnit.METER)*pos.getX(DistanceUnit.METER)+
                                       pos.getY(DistanceUnit.METER)*pos.getY(DistanceUnit.METER))< .21)
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
                        currentState = States.AIMATTARGET;
                    }
                    break;
                case AIMATTARGET:
                    TargetPosition aprilTagLocation = robot.limeLight.getTargetPosition(robot.imu.getRobotYawPitchRollAngles().getYaw(), LimeLight.RED_TARGET_ID);
                    // If the tagLocation isn't valid then the tag isn't on the FOV
                    if(!aprilTagLocation.isValid) {
                        aimRx = 0.5;
                    }
                    // x location should be positive since the cross hair will be to the left of
                    //  the target
                    else if(aprilTagLocation.x > 0.5){
                        aimRx = robot.limeLight.limelight_aim_proportional(aprilTagLocation.x);
                    }
                    // Aiming is finished now shoot.
                    else {
                        aimRx = 0.0;
                        if(currentMotif == LimeLight.Motif.PPG ) {
                            currentState = States.SHOOTPPG;
                        }
                        else if(currentMotif == LimeLight.Motif.PGP){
                            currentState = States.SHOOTPGP;
                        }
                        // Catch if GPP or UNKNOWN
                        else {
                            currentState = States.SHOOTGPP;
                        }
                    }
                    // Handle drive controls using DriveSubsystem
                    robot.driveSubsystem.handleDriveInput(
                            0.0, 0.0,
                            aimRx,
                            0.0,0.0);
                    break;
                case SHOOTGPP:
                    // Wait for shooting to finish
                    if (robot.autonomousSubsystem.shootGPPMotif(robot)) {
                        currentState = States.FINALPOSITION;
                    }
                    break;

                case SHOOTPGP:
                    // Wait for shooting to finish
                    if (robot.autonomousSubsystem.shootPGPMotif(robot)) {
                        currentState = States.FINALPOSITION;
                    }
                    break;

                case SHOOTPPG:
                    // Wait for shooting to finish
                    if (robot.autonomousSubsystem.shootPPGMotif(robot)) {
                        currentState = States.FINALPOSITION;
                    }
                    break;
                case TURNTOFINALPOSITION:
                    if(robot.driveSubsystem.getHeading() > 25.0){
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
                        currentState = States.FINALPOSITION;
                    }
                    break;
                case FINALPOSITION:
                    Pose2D finalPos = robot.pinpoint.getPosition();
                    // Position is returned in Meters
                    if(Math.sqrt(finalPos.getX(DistanceUnit.METER)*finalPos.getX(DistanceUnit.METER)+
                            finalPos.getY(DistanceUnit.METER)*finalPos.getY(DistanceUnit.METER))< 1.2)
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