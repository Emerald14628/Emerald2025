package org.firstinspires.ftc.teamcode;
//comment

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TargetPosition;

@Autonomous
public class Blue3Point extends LinearOpMode {

    enum States{
        AIMATTARGET, SHOOTGPP, SHOOTPGP, SHOOTPPG, FINALPOSITION, END
    }
    private RobotHardware robot;
    private States currentState = States.AIMATTARGET;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        // Variables for button state handling
        boolean lastDpadDownState = false;
        LimeLight.Motif currentMotif = LimeLight.Motif.UNKNOWN;

        robot.driveSubsystem.calibrateIMU();
        robot.pinpoint.resetPosAndIMU();
        robot.colorSubsystem.Update();
        robot.limeLight.start();
        waitForStart();

        if (isStopRequested()) return;
        // Prepare the sound players
        boolean isShootingActive= false;
        double aimRx = 0;
        TargetPosition aprilTagLocation= new TargetPosition();
        robot.pinpoint.initialize();


        if (!isShootingActive) {
            robot.shooterSubsystem.activateHogWheel(ShooterSubsystem.HogWheelPower.POWER_3);
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
                case AIMATTARGET:
                    aprilTagLocation = robot.limeLight.getTargetPosition(robot.imu.getRobotYawPitchRollAngles().getYaw(), LimeLight.BLUE_TARGET_ID);
                    // If the tagLocation isn't valid then the tag isn't on the FOV
                    if(!aprilTagLocation.isValid) {
                        aimRx = -0.25;
                    }
                    // x location should be negative since the cross hair will be to the right of
                    //  the target
                    else if(aprilTagLocation.x < -0.25 || aprilTagLocation.x > 0.25){
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
            telemetry.addData("aimrx", aimRx);
            telemetry.addData("Tag Valid", aprilTagLocation.isValid);
            telemetry.addData("tx", aprilTagLocation.x);
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