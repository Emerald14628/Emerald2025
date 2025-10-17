package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveSubsystem {
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final IMU imu;
    private boolean fieldCentric = true;

    // Complementary filter variables
    private double filteredHeading = 0.0;
    private double lastHeading = 0.0;
    private final double ALPHA = 0.95; // Filter coefficient (adjust between 0 and 1)
    private final ElapsedTime timer = new ElapsedTime();
    private double lastUpdateTime = 0;
    private double driftRate = 0.0; // Degrees per second
    private static final double DRIFT_THRESHOLD = 0.1; // Degrees per second

    public DriveSubsystem(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, IMU imu) {
        this.frontLeftMotor = frontLeft;
        this.backLeftMotor = backLeft;
        this.frontRightMotor = frontRight;
        this.backRightMotor = backRight;
        this.imu = imu;
        timer.reset();
    }

    public void setFieldCentric(boolean enabled) {
        fieldCentric = enabled;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void handleDriveInput(double y, double x, double rx, double leftTrigger, double rightTrigger) {
        // Check IMU status first
        if (!isImuHealthy()) {
            // Fallback to robot-centric mode if IMU is unhealthy
            fieldCentric = false;
        }

        // If no movement input, don't apply any power regardless of trigger state
        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(rx) < 0.1) {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            return;
        }

        double maxPower = calculateMaxPower(leftTrigger, rightTrigger);

        // Apply speed modifications
        y = y * maxPower;
        x = -x * maxPower;
        rx = -rx * maxPower;

        // Get the filtered robot heading
        double botHeading = getFilteredHeading();

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
        double frontRightPower = -(y - x - rx) / denominator;
        double backRightPower = -(y + x - rx) / denominator;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // Update heading tracking
        updateHeadingTracking();
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

    private boolean isImuHealthy() {
        try {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return !orientation.getYaw(AngleUnit.DEGREES).isNaN();
        } catch (Exception e) {
            return false;
        }
    }

    private double getFilteredHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw(AngleUnit.RADIANS);

        // Apply complementary filter
        filteredHeading = ALPHA * (filteredHeading + getAngularVelocity() * getDeltaTime())
                         + (1 - ALPHA) * currentHeading;

        return filteredHeading;
    }

    private double getAngularVelocity() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    private double getDeltaTime() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        return dt;
    }

    private void updateHeadingTracking() {
        double currentHeading = getHeading();
        double deltaTime = getDeltaTime();

        if (deltaTime > 0) {
            double instantDrift = (currentHeading - lastHeading) / deltaTime;
            // Update drift rate with exponential moving average
            driftRate = 0.95 * driftRate + 0.05 * instantDrift;

            // Compensate for drift if it exceeds threshold
            if (Math.abs(driftRate) > DRIFT_THRESHOLD) {
                filteredHeading -= driftRate * deltaTime;
            }
        }

        lastHeading = currentHeading;
    }

    /**
     * Calibrates the IMU by resetting its heading to zero.
     * Call this method when the robot is placed in its starting orientation.
     */
    public void calibrateIMU() {
        imu.resetYaw();
        filteredHeading = 0.0;
        lastHeading = 0.0;
        driftRate = 0.0;
        timer.reset();
        lastUpdateTime = 0;
    }

    /**
     * Gets the current heading of the robot in degrees
     * @return heading in degrees (-180 to 180)
     */
    public double getHeading() {
        if (!isImuHealthy()) {
            return filteredHeading;
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void addMotorPowersToTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("FL Power", String.format("%.2f", frontLeftMotor.getPower()));
        telemetry.addData("FR Power", String.format("%.2f", frontRightMotor.getPower()));
        telemetry.addData("BL Power", String.format("%.2f", backLeftMotor.getPower()));
        telemetry.addData("BR Power", String.format("%.2f", backRightMotor.getPower()));
        telemetry.addData("IMU Health", isImuHealthy() ? "OK" : "WARNING");
        telemetry.addData("Drift Rate", String.format("%.3f°/s", driftRate));
    }
}
