package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class AprilTagLimeLightTest extends OpMode {
    private Limelight3A limelight;

    private IMU  imu;
    @Override
    public void init() {
     limelight = hardwareMap.get(Limelight3A.class, "limelight");
     limelight.pipelineSwitch(0);
     imu = hardwareMap.get(IMU.class, "imu");
     RevHubOrientationOnRobot orientation =
             new RevHubOrientationOnRobot(
                     RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                     RevHubOrientationOnRobot.UsbFacingDirection.UP);
     imu.initialize(new IMU.Parameters(orientation));
    }

    @Override
    public void start() {
        limelight.start ();
    }

    @Override
    public void loop() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose" , botPose.toString());
            telemetry.addData("Yaw" , botPose.getOrientation().getYaw());

            // Get all detected AprilTags
            java.util.List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("AprilTag ID", fr.getFiducialId());
                telemetry.addData("AprilTag Family", fr.getFamily());
                telemetry.addData("AprilTag X", "%.2f", fr.getTargetXDegrees());
                telemetry.addData("AprilTag Y", "%.2f", fr.getTargetYDegrees());
            }
        }
        telemetry.update();
    }


}
