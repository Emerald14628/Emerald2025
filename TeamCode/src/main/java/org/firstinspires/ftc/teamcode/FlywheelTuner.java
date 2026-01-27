package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Flywheel PIDF Tuner")
public class FlywheelTuner extends LinearOpMode {
    DcMotorEx motor1, motor2;
    double p = 0.0, f = 0.0; // Starting default values
    double highVelocity = 1458;
    double lowVelocity = 500;
    double targetVelocity = highVelocity; // Ticks per second (adjust based on your RPM)

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "HogWheel1");
        motor2 = hardwareMap.get(DcMotorEx.class, "HogWheel2");

        // Reverse one motor since they are mechanically linked in opposite directions
        motor2.setDirection(DcMotorEx.Direction.REVERSE);

        // Use RUN_USING_ENCODER to enable built-in PIDF
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        boolean yWasPressed = false;
        boolean bWasPressed = false;
        boolean dPadRightPressed = false;
        boolean dPadLeftPressed = false;
        boolean dPadUpPressed = false;
        boolean dPadDownPressed = false;
        waitForStart();

        while (opModeIsActive()) {


            // ADJUSTING VALUES WITH GAMEPAD 1
            // D-pad Up/Down to adjust P
            if (gamepad1.y && !yWasPressed) {
                if (targetVelocity == highVelocity) {
                    targetVelocity = lowVelocity;
                } else {
                    targetVelocity = highVelocity;
                }
            }

            if (gamepad1.b && !bWasPressed) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if (gamepad1.dpad_up && !dPadUpPressed) p += stepSizes[stepIndex];

            if (gamepad1.dpad_down && !dPadDownPressed) p -= stepSizes[stepIndex];

            // D-pad Left/Right to adjust F
            if (gamepad1.dpad_right && !dPadRightPressed) f += stepSizes[stepIndex];
            if (gamepad1.dpad_left && !dPadLeftPressed) f -= stepSizes[stepIndex];

            yWasPressed = gamepad1.y;
            bWasPressed = gamepad1.b;
            dPadRightPressed = gamepad1.dpad_right;
            dPadLeftPressed = gamepad1.dpad_left;
            dPadUpPressed = gamepad1.dpad_up;
            dPadDownPressed = gamepad1.dpad_down;

            // Update motor PIDF coefficients
            motor1.setVelocityPIDFCoefficients(p, 0, 0, f);
            motor2.setVelocityPIDFCoefficients(p, 0, 0, f);

            // Command velocity

            motor1.setVelocity(targetVelocity);
            motor2.setVelocity(targetVelocity);

            double velocity1 = motor1.getVelocity();
            double velocity2 = motor2.getVelocity();
            double error1 = targetVelocity - velocity1;
            double error2 = targetVelocity - velocity2;

            // Telemetry for real-time monitoring
            telemetry.addData("Target Velocity (Y Button)", targetVelocity);
            telemetry.addData("Actual Velocity 1", velocity1);
            telemetry.addData("Actual Velocity 2", velocity2);
            telemetry.addData("Error1", "%.2f", error1);
            telemetry.addData("Error2", "%.2f", error2);
            telemetry.addData("Current P (Up/Down)", p);
            telemetry.addData("Current F (Left/Right)", f);
            telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
            telemetry.update();
        }
    }
}



