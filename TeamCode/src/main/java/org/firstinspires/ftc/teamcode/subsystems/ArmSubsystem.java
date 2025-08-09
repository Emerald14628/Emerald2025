package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Constants;

public class ArmSubsystem {
    private final DcMotor armMotor;
    private final DcMotor udarmMotor;
    private final TouchSensor armLimit;

    private boolean buttonpress = false;
    private boolean button0 = false;
    private boolean buttonpress2 = false;
    private int limitedPos;
    private int maxPos;

    public ArmSubsystem(DcMotor armMotor, DcMotor udarmMotor, TouchSensor armLimit) {
        this.armMotor = armMotor;
        this.udarmMotor = udarmMotor;
        this.armLimit = armLimit;

        initializePositions();
    }

    private void initializePositions() {
        limitedPos = armMotor.getCurrentPosition() + Constants.ViperslideConstants.ViperSlideLimitedPos;
        maxPos = armMotor.getCurrentPosition() + Constants.ViperslideConstants.ViperSlideMaxPos;
    }

    public void handleArmMovement(boolean dpadUp, boolean dpadDown) {
        updateLimitSwitchStates();

        int currentPosition = armMotor.getCurrentPosition();

        if (dpadUp && currentPosition >= maxPos) {
            if (currentPosition >= limitedPos || buttonpress) {
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor.setPower(-1);
            } else {
                armMotor.setTargetPosition(currentPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else if (dpadDown) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(1);
        } else {
            armMotor.setPower(0);
        }
    }

    private void updateLimitSwitchStates() {
        if (armLimit.getValue() == 1 && !buttonpress) {
            buttonpress = true;
        } else if (armLimit.getValue() == 0 && buttonpress && !button0) {
            button0 = true;
        } else if (button0 && armLimit.getValue() == 1) {
            buttonpress2 = true;
        } else if (armLimit.getValue() == 0 && buttonpress && button0 && buttonpress2) {
            resetStates();
        }
    }

    public void resetStates() {
        buttonpress = false;
        button0 = false;
        buttonpress2 = false;
    }

    public void resetEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        udarmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
