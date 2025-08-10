package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveSubsystem {
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final IMU imu;
    private boolean fieldCentric = true;

    public DriveSubsystem(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, IMU imu) {
        this.frontLeftMotor = frontLeft;
        this.backLeftMotor = backLeft;
        this.frontRightMotor = frontRight;
        this.backRightMotor = backRight;
        this.imu = imu;
    }

    public void setFieldCentric(boolean enabled) {
        fieldCentric = enabled;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void handleDriveInput(double y, double x, double rx, double leftTrigger, double rightTrigger) {
        double maxPower = calculateMaxPower(leftTrigger, rightTrigger);

        // Apply speed modifications
        y = y * maxPower;
        x = x * maxPower;
        rx = rx * maxPower;

        // Get the robot's heading from the IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);

        // If field centric is enabled, adjust input vectors based on robot heading
        if (fieldCentric) {
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            x = rotX;
            y = rotY;
        }

        // Calculate motor powers using mecanum drive kinematics
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
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

    /**
     * Calibrates the IMU by resetting its heading to zero.
     * Call this method when the robot is placed in its starting orientation.
     */
    public void calibrateIMU() {
        imu.resetYaw();
    }

    /**
     * Gets the current heading of the robot in degrees
     * @return heading in degrees (-180 to 180)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
