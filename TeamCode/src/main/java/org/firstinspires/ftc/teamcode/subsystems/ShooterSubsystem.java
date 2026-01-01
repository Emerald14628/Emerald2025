package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {  private final CRServo LeftShooter;
    private final CRServo RightShooter;
    private final DcMotor HogWheel1;
    private final DcMotor HogWheel2;
    private final Servo ShooterLeftAngle;
    private final Servo ShooterRightAngle;
    private final double[] ShooterAngles = {0.5, 0.75, 1.0, 0.75, 0.5, 0.25, 0.0, 0.25};
    private int CurrentShooterState;
    public ShooterSubsystem(HardwareMap hardwareMap) {
        LeftShooter = hardwareMap.get(CRServo.class, "LeftShooter");
        RightShooter = hardwareMap.get(CRServo.class, "RightShooter");
        HogWheel1 = hardwareMap.get(DcMotor.class, "HogWheel1");
        HogWheel2 = hardwareMap.get(DcMotor.class, "HogWheel2");
        ShooterLeftAngle = hardwareMap.get(Servo.class, "ShooterLeftAngle");
        ShooterRightAngle = hardwareMap.get(Servo.class, "ShooterRightAngle");
        CurrentShooterState = 0;
        // Set motor directions
        LeftShooter.setDirection(DcMotor.Direction.FORWARD);
        RightShooter.setDirection(DcMotor.Direction.REVERSE);
        HogWheel1.setDirection(DcMotor.Direction.FORWARD);
        HogWheel2.setDirection(DcMotor.Direction.REVERSE);
    }

    public void pushArtifactRight() {
        RightShooter.setPower(-1.0);
    }
    public void ejectArtifactRight() {
        RightShooter.setPower(1.0);
    }
    public void stopArtifactRight() {
        RightShooter.setPower(0);
    }
    public void pushArtifactLeft() {
        LeftShooter.setPower(-1.0);
    }
        public void ejectArtifactLeft() {
            LeftShooter.setPower(1.0);
        }
    public void stopArtifactLeft() {
        LeftShooter.setPower(0);
    }
    public void activateHogWheel() {
        HogWheel1.setPower(1.0);
        HogWheel2.setPower(1.0);
    }
    public void stopHogWheel() {
        HogWheel1.setPower(0);
        HogWheel2.setPower(0);
    }
    public void changeShooterAngleFoward() {
        if (CurrentShooterState == 7)
        {
            CurrentShooterState = 0;
        }
       else {
           CurrentShooterState ++;
       }
        ShooterRightAngle.setPosition(ShooterAngles[CurrentShooterState] );
       ShooterLeftAngle.setPosition(ShooterAngles[CurrentShooterState] );
    }

    public void changeShooterAngleBackward() {
        if (CurrentShooterState == 0){
            CurrentShooterState = 7;
        }
        else
        {
            CurrentShooterState --;
        }
        ShooterRightAngle.setPosition(ShooterAngles[CurrentShooterState] );
        ShooterLeftAngle.setPosition(ShooterAngles[CurrentShooterState] );
    }

    public void stopShooting() {
        LeftShooter.setPower(0);
        RightShooter.setPower(0);
        HogWheel1.setPower(0);
        HogWheel2.setPower(0);
        ShooterLeftAngle.setPosition(0);
        ShooterRightAngle.setPosition(0);
    }
}
