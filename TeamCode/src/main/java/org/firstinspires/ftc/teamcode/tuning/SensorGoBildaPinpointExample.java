/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobildasensors.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */

@TeleOp(name="goBILDA Pinpoint Example", group="Linear OpMode")
//@Disabled

public class SensorGoBildaPinpointExample extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    // Drive motors
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    // Constants for square pattern
    private static final double SQUARE_SIZE_MM = 609.6; // 2 feet = 24 inches = 609.6 mm
    private static final double DRIVE_SPEED = 0.3;
    private static final double TURN_SPEED = 0.25;
    private static final double POSITION_TOLERANCE = 25.0; // 25mm tolerance
    private static final double HEADING_TOLERANCE = 5.0; // 5 degrees tolerance

    double oldTime = 0;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // Initialize drive motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);


            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            if (gamepad1.x){
                // Drive a 2-foot square pattern
                runTwoFootSquare();
            }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();

        }
    }

    /**
     * Execute a 2-foot square pattern using odometry
     * Press gamepad1.x to trigger this
     */
    private void runTwoFootSquare() {
        telemetry.addData("Status", "Starting 2-foot square pattern");
        telemetry.update();
        sleep(300); // Brief pause before starting

        // Drive the square pattern
        // Side 1: Move forward 2 feet
        driveToPosition(SQUARE_SIZE_MM, 0, 0);
        sleep(500);

        // Turn 90 degrees left (counterclockwise)
        turnToHeading(90);
        sleep(500);

        // Side 2: Move forward 2 feet (now moving left relative to start)
        driveToPosition(SQUARE_SIZE_MM, SQUARE_SIZE_MM, 90);
        sleep(500);

        // Turn 90 degrees left (total 180 degrees)
        turnToHeading(180);
        sleep(500);

        // Side 3: Move forward 2 feet (now moving backward relative to start)
        driveToPosition(0, SQUARE_SIZE_MM, 180);
        sleep(500);

        // Turn 90 degrees left (total 270 degrees)
        turnToHeading(270);
        sleep(500);

        // Side 4: Move forward 2 feet (returning to start)
        driveToPosition(0, 0, 270);
        sleep(500);

        // Final turn to face original direction
        turnToHeading(0);

        telemetry.addData("Status", "Square complete!");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Drive to a target position using odometry feedback
     */
    private void driveToPosition(double targetX, double targetY, double targetHeading) {
        while (opModeIsActive()) {
            // Update odometry
            odo.update();

            // Get current position
            Pose2D currentPos = odo.getPosition();
            double currentX = currentPos.getX(DistanceUnit.MM);
            double currentY = currentPos.getY(DistanceUnit.MM);

            // Calculate errors
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double distance = Math.sqrt(errorX * errorX + errorY * errorY);

            // Check if we've reached the target
            if (distance < POSITION_TOLERANCE) {
                stopDriving();
                break;
            }

            // Calculate angle to target
            double angleToTarget = Math.toDegrees(Math.atan2(errorY, errorX));
            double currentHeading = currentPos.getHeading(AngleUnit.DEGREES);

            // Calculate heading error
            double headingError = normalizeAngle(angleToTarget - currentHeading);

            // Simple proportional control
            double forward = DRIVE_SPEED * Math.cos(Math.toRadians(headingError));
            double strafe = DRIVE_SPEED * Math.sin(Math.toRadians(headingError));
            double turn = headingError * 0.02; // Proportional turn correction

            // Limit turn rate
            turn = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, turn));

            // Drive with mecanum wheels
            driveMecanum(forward, strafe, turn);

            // Telemetry
            telemetry.addData("Target", "X: %.1f, Y: %.1f", targetX, targetY);
            telemetry.addData("Current", "X: %.1f, Y: %.1f", currentX, currentY);
            telemetry.addData("Distance to target", "%.1f mm", distance);
            telemetry.addData("Heading", "%.1f degrees", currentHeading);
            telemetry.update();
        }
    }

    /**
     * Turn to a specific heading using odometry feedback
     */
    private void turnToHeading(double targetHeading) {
        while (opModeIsActive()) {
            // Update odometry
            odo.update();

            // Get current heading
            Pose2D currentPos = odo.getPosition();
            double currentHeading = currentPos.getHeading(AngleUnit.DEGREES);

            // Calculate heading error
            double headingError = normalizeAngle(targetHeading - currentHeading);

            // Check if we've reached the target
            if (Math.abs(headingError) < HEADING_TOLERANCE) {
                stopDriving();
                break;
            }

            // Proportional turn control
            double turn = headingError * 0.015;
            turn = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, turn));

            // Turn in place
            driveMecanum(0, 0, turn);

            // Telemetry
            telemetry.addData("Target Heading", "%.1f degrees", targetHeading);
            telemetry.addData("Current Heading", "%.1f degrees", currentHeading);
            telemetry.addData("Heading Error", "%.1f degrees", headingError);
            telemetry.update();
        }
    }

    /**
     * Mecanum drive method
     */
    private void driveMecanum(double forward, double strafe, double turn) {
        double frontLeftPower = forward + strafe + turn;
        double backLeftPower = forward - strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backRightPower = forward + strafe - turn;

        // Normalize powers
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                                   Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Stop all drive motors
     */
    private void stopDriving() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * Normalize angle to -180 to 180 degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
