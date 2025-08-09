package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestOpClass extends LinearOpMode {

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, armMotor, udarmMotor;
    private CRServo clawServo, wristServo, wristSpinServo;
    private TouchSensor armLimit;

    // State
    private int udarmPos, viperslidepos, limitedPos, maxPos;
    private boolean buttonpress, setViperPOS;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        resetEncoders();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            updateDrive();
            updateArm();
            updateUdarm();
            updateServos();
            updateTelemetry();
        }
    }

    private void initHardware() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        udarmMotor = hardwareMap.dcMotor.get("udarmMotor");

        clawServo = hardwareMap.crservo.get("clawServo");
        wristServo = hardwareMap.crservo.get("wristServo");
        wristSpinServo = hardwareMap.crservo.get("wristSpinServo");
        armLimit = hardwareMap.touchSensor.get("armLimit");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void resetEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        udarmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        udarmPos = udarmMotor.getCurrentPosition();
        viperslidepos = armMotor.getCurrentPosition();
        limitedPos = viperslidepos + Constants.ViperslideConstants.ViperSlideLimitedPos;
        maxPos = viperslidepos + Constants.ViperslideConstants.ViperSlideMaxPos;

        buttonpress = false;
        setViperPOS = false;
    }

    private void updateDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double maxPower = Constants.MotorConstants.driveSpeed;

        if (gamepad1.left_trigger != 0) {
            maxPower = Constants.MotorConstants.driveSpeed +
                    ((1 - Constants.MotorConstants.driveSpeed) * gamepad1.left_trigger);
        }
        if (gamepad1.right_trigger != 0) {
            maxPower = Constants.MotorConstants.driveSpeed - Constants.MotorConstants.driveSpeed * gamepad1.right_trigger;
            if (maxPower < 0.1) maxPower = 0.1;
        }

        y *= maxPower;
        x *= maxPower;
        rx *= maxPower;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void updateArm() {
        viperslidepos = armMotor.getCurrentPosition();
        if (armLimit.getValue() == 1) {
            buttonpress = true;
            setViperPOS = false;
        } else {
            buttonpress = false;
            if (armMotor.getCurrentPosition() < limitedPos) {
                if (!setViperPOS) {
                    setViperPOS = true;
                    armMotor.setTargetPosition(limitedPos);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1);
                }
            } else {
                if (setViperPOS) {
                    armMotor.setPower(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setViperPOS = false;
                }
            }
        }

        if (gamepad2.dpad_up && armMotor.getCurrentPosition() >= maxPos) {
            if (armMotor.getCurrentPosition() >= limitedPos || buttonpress) {
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor.setPower(-1);
            } else {
                armMotor.setTargetPosition(viperslidepos);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else if (gamepad2.dpad_down) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(1);
        } else {
            if (!setViperPOS) {
                armMotor.setTargetPosition(viperslidepos);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

    private void updateUdarm() {
        if (gamepad2.left_bumper) {
            udarmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            udarmMotor.setPower(0.5);
            udarmPos = udarmMotor.getCurrentPosition();
        } else if (gamepad2.right_bumper) {
            udarmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            udarmMotor.setPower(-0.5);
            udarmPos = udarmMotor.getCurrentPosition();
        } else {
            udarmMotor.setTargetPosition(udarmPos);
            udarmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void updateServos() {
        double clawSpeed = 0;
        if (gamepad2.x) clawSpeed = 0.5;
        if (gamepad2.triangle) clawSpeed = -0.5;
        clawServo.setPower(clawSpeed);

        double wristSpinSpeed = 0;
        if (gamepad2.a) wristSpinSpeed = 0.5;
        if (gamepad2.b) wristSpinSpeed = -0.5;
        wristSpinServo.setPower(wristSpinSpeed);

        double wristSpeed = 0;
        if (gamepad2.left_trigger >= 0.2) wristSpeed = -0.3;
        if (gamepad2.right_trigger >= 0.2) wristSpeed = 0.3;
        wristServo.setPower(wristSpeed);
    }

    private void updateTelemetry() {
        telemetry.addData("Viper Slide Postion", armMotor.getCurrentPosition());
        telemetry.addData("armLimit", armLimit.getValue());
        telemetry.addData("buttonpress", buttonpress);
        telemetry.addData("limitedPos", limitedPos);
        telemetry.addData("MaxPos", maxPos);
        telemetry.addData("setViperPOS", setViperPOS);
        telemetry.addData("udarmMotorPOS", udarmMotor.getCurrentPosition());
        telemetry.update();
    }
}