package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {private final DcMotor intakeMotor;

    public IntakeSubsystem(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void intakeArtifact(){
        intakeMotor.setPower(1);
    }
public void ejectArtifact(){
        intakeMotor.setPower(-1);
    }
    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
