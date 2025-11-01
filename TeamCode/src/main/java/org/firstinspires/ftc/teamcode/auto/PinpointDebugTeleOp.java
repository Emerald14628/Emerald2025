package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobildasensors.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Pinpoint Debug - Error Details", group="tuning")
public class PinpointDebugTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Attempting to initialize Pinpoint...");
        telemetry.update();

        GoBildaPinpointDriver pinpoint = null;
        String errorDetails = "No error";
        String stackTrace = "";

        try {
            // Try to get the Pinpoint device from hardware map
            telemetry.addLine("Step 1: Getting device from hardware map...");
            telemetry.update();

            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

            telemetry.addLine("Step 1: SUCCESS - Device retrieved");
            telemetry.update();
            Thread.sleep(500);

            // Try to configure encoder resolution
            telemetry.addLine("Step 2: Setting encoder resolution...");
            telemetry.update();

            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            telemetry.addLine("Step 2: SUCCESS - Resolution set");
            telemetry.update();
            Thread.sleep(500);

            // Try to configure encoder directions
            telemetry.addLine("Step 3: Setting encoder directions...");
            telemetry.update();

            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                         GoBildaPinpointDriver.EncoderDirection.FORWARD);

            telemetry.addLine("Step 3: SUCCESS - Directions set");
            telemetry.update();
            Thread.sleep(500);

            // Try to set offsets
            telemetry.addLine("Step 4: Setting offsets...");
            telemetry.update();

            pinpoint.setOffsets(0, 0);

            telemetry.addLine("Step 4: SUCCESS - Offsets set");
            telemetry.update();
            Thread.sleep(500);

            // Try to reset
            telemetry.addLine("Step 5: Resetting position and IMU...");
            telemetry.update();

            pinpoint.resetPosAndIMU();

            telemetry.addLine("Step 5: SUCCESS - Reset complete");
            telemetry.update();
            Thread.sleep(500);

            errorDetails = "ALL STEPS SUCCESSFUL!";

        } catch (IllegalArgumentException e) {
            errorDetails = "IllegalArgumentException: " + e.getMessage();
            stackTrace = getStackTraceString(e);
        } catch (Exception e) {
            errorDetails = e.getClass().getSimpleName() + ": " + e.getMessage();
            stackTrace = getStackTraceString(e);
        }

        // Display results
        telemetry.clear();
        telemetry.addLine("=== INITIALIZATION RESULTS ===");
        telemetry.addLine();
        telemetry.addData("Status", errorDetails);
        telemetry.addLine();

        if (!stackTrace.isEmpty()) {
            telemetry.addLine("=== STACK TRACE ===");
            // Split stack trace into lines and show first 10
            String[] lines = stackTrace.split("\n");
            for (int i = 0; i < Math.min(lines.length, 10); i++) {
                telemetry.addLine(lines[i]);
            }
            if (lines.length > 10) {
                telemetry.addLine("... (truncated)");
            }
        }

        telemetry.addLine();
        telemetry.addLine("Press Start to continue or Stop to exit");
        telemetry.update();

        waitForStart();

        if (pinpoint != null && errorDetails.equals("ALL STEPS SUCCESSFUL!")) {
            telemetry.addLine("Pinpoint initialized successfully!");
            telemetry.addLine("Reading encoder values...");
            telemetry.update();

            while (opModeIsActive() && !isStopRequested()) {
                pinpoint.update();

                int encX = pinpoint.getEncoderX();
                int encY = pinpoint.getEncoderY();

                telemetry.addData("Encoder X", encX);
                telemetry.addData("Encoder Y", encY);
                telemetry.addData("X changing?", encX != 0 ? "YES" : "NO");
                telemetry.addData("Y changing?", encY != 0 ? "YES" : "NO");
                telemetry.update();
            }
        } else {
            telemetry.addLine("Initialization failed. Review error above.");
            telemetry.update();

            while (opModeIsActive() && !isStopRequested()) {
                sleep(100);
            }
        }
    }

    private String getStackTraceString(Throwable e) {
        StringBuilder sb = new StringBuilder();
        sb.append(e.toString()).append("\n");

        StackTraceElement[] elements = e.getStackTrace();
        for (int i = 0; i < Math.min(elements.length, 15); i++) {
            sb.append("  at ").append(elements[i].toString()).append("\n");
        }

        if (e.getCause() != null) {
            sb.append("Caused by: ").append(e.getCause().toString()).append("\n");
            StackTraceElement[] causeElements = e.getCause().getStackTrace();
            for (int i = 0; i < Math.min(causeElements.length, 10); i++) {
                sb.append("  at ").append(causeElements[i].toString()).append("\n");
            }
        }

        return sb.toString();
    }
}
