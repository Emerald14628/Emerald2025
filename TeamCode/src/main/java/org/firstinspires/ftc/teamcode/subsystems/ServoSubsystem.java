package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

public class ServoSubsystem {
    private final CRServo clawServo;
    private final CRServo wristServo;
    private final CRServo wristSpinServo;

    public ServoSubsystem(CRServo clawServo, CRServo wristServo, CRServo wristSpinServo) {
        this.clawServo = clawServo;
        this.wristServo = wristServo;
        this.wristSpinServo = wristSpinServo;
    }

    public void handleClawControl(boolean open, boolean close) {
        if (open) {
            clawServo.setPower(1.0);
        } else if (close) {
            clawServo.setPower(-1.0);
        } else {
            clawServo.setPower(0);
        }
    }

    public void handleWristControl(boolean up, boolean down) {
        if (up) {
            wristServo.setPower(1.0);
        } else if (down) {
            wristServo.setPower(-1.0);
        } else {
            wristServo.setPower(0);
        }
    }

    public void handleWristSpinControl(boolean spinLeft, boolean spinRight) {
        if (spinLeft) {
            wristSpinServo.setPower(1.0);
        } else if (spinRight) {
            wristSpinServo.setPower(-1.0);
        } else {
            wristSpinServo.setPower(0);
        }
    }
}
