package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Test Pedro Path")
public class TestPedroPathAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize MecanumDrive with starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build a square path using the new Roadrunner API
        Action squarePath = drive.actionBuilder(startPose)
                .lineToX(24)  // Drive forward 24 inches
                .turn(Math.toRadians(90))  // Turn 90 degrees
                .lineToY(24)  // Strafe 24 inches
                .turn(Math.toRadians(90))  // Turn 90 degrees
                .lineToX(0)   // Drive back 24 inches
                .turn(Math.toRadians(90))  // Turn 90 degrees
                .lineToY(0)   // Strafe back 24 inches
                .turn(Math.toRadians(90))  // Return to starting angle
                .build();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Execute the path
        Actions.runBlocking(squarePath);

        // Display final position
        telemetry.addData("Status", "Path Complete");
        telemetry.addData("X Position", drive.pose.position.x);
        telemetry.addData("Y Position", drive.pose.position.y);
        telemetry.addData("Heading", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        // Keep the OpMode active so we can see telemetry
        while (opModeIsActive() && !isStopRequested()) {
            sleep(100);
        }
    }
}
