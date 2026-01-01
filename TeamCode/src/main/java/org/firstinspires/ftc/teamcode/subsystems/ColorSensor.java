package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor {


    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hwMp, String sensorName) {
        colorSensor = hwMp.get(NormalizedColorSensor.class, sensorName);
        colorSensor.setGain(15);
    }


    public ColorSensor.DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor .getNormalizedColors ();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry .addData("red", normRed);
        telemetry .addData("green", normGreen);
        telemetry .addData("blue", normBlue);
        telemetry .addData("alpha", colors.alpha);

        float  hue = JavaUtil.colorToHue(colors.toColor());
        telemetry.addData("Hue", hue);
        if (hue >= 81 && hue <= 170 ) {
                return  DetectedColor.GREEN;
            }
            else if (hue >= 175 && hue <= 300 ) {
            return  DetectedColor.PURPLE;
            }

        return  DetectedColor.UNKNOWN;
        }
}