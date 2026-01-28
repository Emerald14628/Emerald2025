package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class BlikinColorDevice {
    private RevBlinkinLedDriver blinkin;

    public void init(HardwareMap hwMp, String sensorName) {
        blinkin = hwMp.get(RevBlinkinLedDriver.class, sensorName);
    }
    public void setAllGreen () {

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
}
    public void setAllPurple () {

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }
    public void setAllBlue () {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public void setAllRed () {

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
}
    public void setLeftGreenRightPurple () {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2);
    }
    public void setLeftPurpleRightGreen () {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
    }

    public void setLeftGreenRightUnknown(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE);
    }
    public void setLeftPurpleRightUnknown(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
    }

    public void setLeftUnknownRightGreen(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
    }
    public void setLeftUnknownRightPurple(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER);
    }
    public void setLarsonRedScanner(){
        blinkin.setPattern((RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED));
    }
}