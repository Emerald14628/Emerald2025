package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem {
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;

    public DriveSubsystem(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        this.frontLeftMotor = frontLeft;
        this.backLeftMotor = backLeft;
        this.frontRightMotor = frontRight;
        this.backRightMotor = backRight;
    }

    public void handleDriveInput(double y, double x, double rx, double leftTrigger, double rightTrigger) {
        double maxPower = calculateMaxPower(leftTrigger, rightTrigger);

        // Apply speed modifications
        y = y * maxPower;
        x = x * maxPower * 1.1; // Keeping the original 1.1 multiplier for x
        rx = rx * maxPower;

        // Calculate motor powers using the denominator method
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    private double calculateMaxPower(double leftTrigger, double rightTrigger) {
        double maxPower = Constants.MotorConstants.driveSpeed;

        if (leftTrigger != 0) {
            maxPower = Constants.MotorConstants.driveSpeed +
                    ((1 - Constants.MotorConstants.driveSpeed) * leftTrigger);
        }

        if (rightTrigger != 0) {
            maxPower = Constants.MotorConstants.driveSpeed - Constants.MotorConstants.driveSpeed * rightTrigger;
            if (maxPower < 0.1) {
                maxPower = 0.1;
            }
        }

        return maxPower;
    }

    private void setMotorPowers(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(-fl);
        backLeftMotor.setPower(-bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }
}
