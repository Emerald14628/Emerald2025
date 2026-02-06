package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {
    private final DcMotor intakeMotor1;
    private final DcMotor intakeMotor2;
    public IntakeSubsystem(DcMotor intakeMotor1, DcMotor intakeMotor2) {
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
    }

    public void intakeArtifactStage1(){
        intakeMotor1.setPower(1);
    }

    public void intakeArtifactStage2(){
        intakeMotor2.setPower(1);
    }

    public void fullIntakeArtifact(){
        intakeMotor1.setPower(1);
        intakeMotor2.setPower(1);
    }
    public void ejectArtifactStage1(){
        intakeMotor1.setPower(-1);
    }
    public void ejectArtifactStage2(){
        intakeMotor2.setPower(-1);
    }

    public void fullEjectArtifact(){
        intakeMotor1.setPower(-1);
        intakeMotor2.setPower(-1);
    }
    public void stopIntake() {
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
    }
}
