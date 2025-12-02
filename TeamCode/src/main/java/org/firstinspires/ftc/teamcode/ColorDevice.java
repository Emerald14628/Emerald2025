package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColorDevice extends OpMode {

    ColorSensor colorSensor1 = new ColorSensor();
    ColorSensor.DetectedColor detectedColor;

    @Override
    public void init() {
colorSensor1.init(hardwareMap, "colorSensor1");
    }

    @Override
    public void loop() {
detectedColor = colorSensor1 .getDetectedColor(telemetry);
  telemetry .addData(  "Color Detected", detectedColor);
    }
}
