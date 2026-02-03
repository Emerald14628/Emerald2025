package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {
    private static class PIDF
    {
        double P;
        double I;
        double D;
        double F;

        public PIDF(double pValue, double iValue, double dValue, double fValue) {
            this.P = pValue;
            this.I = iValue;
            this.D = dValue;
            this.F = fValue;
        }
    }
    private final PIDF[] HogWheelCoefficients = {
            new PIDF(230.4, 0.0, 0.0, 13.9), //
            new PIDF(230.4, 0, 0, 13.9),
            new PIDF(230.4, 0, 0, 13.9)
    };

    private final double[] HogWheel1Speed = {
            1287, 1582, 2300
    };

    private final double[] HogWheel2Speed = {
            1287, 1482, 2100
    };
    public enum HogWheelPower {
        POWER_1,
        POWER_2,
        POWER_3
    }
    private final CRServo LeftShooter;
    private final CRServo RightShooter;
    private final DcMotorEx HogWheel1;
    private final DcMotorEx HogWheel2;
    private final Servo ShooterLeftAngle;
    private final Servo ShooterRightAngle;
    private final double[] ShooterAngles = {0.5, 0.75, 1.0, 0.75, 0.5, 0.25, 0.0, 0.25};
    private int CurrentShooterState;
    public ShooterSubsystem(HardwareMap hardwareMap) {
        LeftShooter = hardwareMap.get(CRServo.class, "LeftShooter");
        RightShooter = hardwareMap.get(CRServo.class, "RightShooter");
        HogWheel1 = hardwareMap.get(DcMotorEx.class, "HogWheel1");
        HogWheel2 = hardwareMap.get(DcMotorEx.class, "HogWheel2");
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
    public void activateHogWheel(double power) {
        HogWheel1.setPower(power);
        HogWheel2.setPower(power);
    }

    public void activateHogWheel(HogWheelPower power){
        HogWheel1.setVelocityPIDFCoefficients(HogWheelCoefficients[power.ordinal()].P,
                                              HogWheelCoefficients[power.ordinal()].I,
                                              HogWheelCoefficients[power.ordinal()].D,
                                              HogWheelCoefficients[power.ordinal()].F);
        HogWheel2.setVelocityPIDFCoefficients(HogWheelCoefficients[power.ordinal()].P,
                                              HogWheelCoefficients[power.ordinal()].I,
                                              HogWheelCoefficients[power.ordinal()].D,
                                              HogWheelCoefficients[power.ordinal()].F);

        // Command velocity

        HogWheel1.setVelocity(HogWheel1Speed[power.ordinal()]);
        HogWheel2.setVelocity(HogWheel2Speed[power.ordinal()]);
    }

    public double gethogwheel1power() {
        return HogWheel1.getPower();
    }
    public double gethogwheel2power() {
        return HogWheel2.getPower();
    }

    public double getHogWheel1RPM() {
        // Get current velocity in ticks per second
        double velocity = HogWheel1.getVelocity();
        // goBILDA 6000 RPM motor: 28 PPR × 4 (quadrature) = 112 ticks per revolution
        double TICKS_PER_REV = 28.0;
        // Convert ticks/second to RPM
        return (velocity / TICKS_PER_REV) * 60.0;
    }

    public double getHogWheel2RPM() {
        // Get current velocity in ticks per second
        double velocity = HogWheel2.getVelocity();
        // goBILDA 6000 RPM motor: 28 PPR × 4 (quadrature) = 112 ticks per revolution
        double TICKS_PER_REV = 28.0;
        // Convert ticks/second to RPM
        return (velocity / TICKS_PER_REV) * 60.0;
    }
    public void stopHogWheel() {
        HogWheel1.setPower(0);
        HogWheel2.setPower(0);
    }
    public void changeShooterAngleForward() {
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
        stopHogWheel();
        ShooterLeftAngle.setPosition(0);
        ShooterRightAngle.setPosition(0);
    }
}
