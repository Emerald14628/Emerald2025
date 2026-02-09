package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class ColorSubsystem {
    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    private static class ColorSensor {
        NormalizedColorSensor colorSensor;

        public void init(HardwareMap hwMp, String sensorName) {
            colorSensor = hwMp.get(NormalizedColorSensor.class, sensorName);
            colorSensor.setGain(15);
        }


        public org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem.DetectedColor getDetectedColor() {
            NormalizedRGBA colors = colorSensor .getNormalizedColors ();
            float  hue = JavaUtil.colorToHue(colors.toColor());

            // An alpha channel closer to 1 means an object is closer
            if(colors.alpha >= 0.65) {
                if (hue >= 81 && hue <= 170) {
                    return org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem.DetectedColor.GREEN;
                } else if (hue >= 175 && hue <= 300) {
                    return org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem.DetectedColor.PURPLE;
                }
            }
            return  org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem.DetectedColor.UNKNOWN;
        }
    }
    private final ColorSensor shooterLeftColorSensor= new ColorSensor();
    private final ColorSensor shooterRightColorSensor= new ColorSensor();
    private final ColorSensor frontLeftColorSensor= new ColorSensor();
    private final ColorSensor frontRightColorSensor= new ColorSensor();
    private final BlikinColorDevice blinkinDevice = new BlikinColorDevice();
    public DetectedColor shooterLeftColor = DetectedColor.UNKNOWN;
    public DetectedColor shooterRightColor = DetectedColor.UNKNOWN;
    public DetectedColor frontLeftColor = DetectedColor.UNKNOWN;
    public DetectedColor frontRightColor = DetectedColor.UNKNOWN;

    public void init(HardwareMap hwMp, String leftShooterName, String rightShooterName, String leftFront, String rightFront, String blinkingName) {
        shooterLeftColorSensor.init(hwMp, leftShooterName);
        shooterRightColorSensor.init(hwMp, rightShooterName);
        frontLeftColorSensor.init(hwMp, leftFront);
        frontRightColorSensor.init(hwMp, rightFront);
        blinkinDevice.init(hwMp, blinkingName);
    }

    public void Update(){
        // Color sensors
        shooterRightColor = shooterRightColorSensor.getDetectedColor();
        shooterLeftColor = shooterLeftColorSensor.getDetectedColor();
        frontRightColor = frontRightColorSensor.getDetectedColor();
        frontLeftColor = frontLeftColorSensor.getDetectedColor();

        // First Case is left color is unknown
        if(shooterLeftColor == DetectedColor.UNKNOWN) {
            // if the right is unknown
            if(shooterRightColor == DetectedColor.UNKNOWN) {
                blinkinDevice.setLarsonRedScanner();
            }
            else if (shooterRightColor == DetectedColor.GREEN){
                blinkinDevice.setLeftUnknownRightGreen();
            }
            else{
                blinkinDevice.setLeftUnknownRightPurple();
            }
        }
        // Second case left is green
        else if(shooterLeftColor == DetectedColor.GREEN){
            // if the right is unknown
            if(shooterRightColor == DetectedColor.UNKNOWN) {
                blinkinDevice.setLeftGreenRightUnknown();
            }
            else if (shooterRightColor == DetectedColor.GREEN){
                blinkinDevice.setAllGreen();
            }
            else{
                blinkinDevice.setLeftGreenRightPurple();
            }
        }
        // last case left if purple
        else{
            // if the right is unknown
            if(shooterRightColor == DetectedColor.UNKNOWN) {
                blinkinDevice.setLeftPurpleRightUnknown();
            }
            else if (shooterRightColor == DetectedColor.GREEN){
                blinkinDevice.setLeftPurpleRightGreen();
            }
            else{
                blinkinDevice.setAllPurple();
            }
        }
    }
}
