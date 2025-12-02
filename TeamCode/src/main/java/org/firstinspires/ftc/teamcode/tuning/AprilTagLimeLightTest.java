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
    private IMU imu;
    private String detectedMotif = "u";
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

        // Always show status first
        telemetry.addData("---STATUS---", "");
        telemetry.addData("Limelight Connected", limelight != null ? "YES" : "NO");

        if (llResult != null) {
            telemetry.addData("Result Valid", llResult.isValid() ? "YES" : "NO");

            if (llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", "%.2f", llResult.getTx());
                telemetry.addData("Ty", "%.2f", llResult.getTy());
                telemetry.addData("Ta", "%.2f", llResult.getTa());

                // Get all detected AprilTags
                java.util.List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                telemetry.addData("---APRILTAGS---", "");
                telemetry.addData("Number of Tags", fiducialResults.size());

                if (fiducialResults.size() == 0) {
                    telemetry.addData("Tag Status", "NO TAGS FOUND");
                    detectedMotif = "NO TAGS";
                } else {
                    int tagCount = 0;
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        tagCount++;
                        int tagId = fr.getFiducialId();

                        telemetry.addData("Tag " + tagCount + " ID", tagId);
                        telemetry.addData("Tag " + tagCount + " Family", fr.getFamily());
                        telemetry.addData("Tag " + tagCount + " X", "%.2f deg", fr.getTargetXDegrees());
                        telemetry.addData("Tag " + tagCount + " Y", "%.2f deg", fr.getTargetYDegrees());

                        // Check AprilTag ID and set motif
                        if (tagId == 23) {
                            detectedMotif = "P P G";
                        } else if (tagId == 22) {
                            detectedMotif = "P G P";
                        } else if (tagId == 21) {
                            detectedMotif = "G P P";
                        } else {
                            detectedMotif = "UNKNOWN TAG: " + tagId;
                        }
                    }
                }
            } else {
                telemetry.addData("Tag Status", "NO VALID RESULTS");
                detectedMotif = "NO DETECTION";
            }
        } else {
            telemetry.addData("Result Valid", "NULL RESULT");
            telemetry.addData("Tag Status", "LIMELIGHT NOT RESPONDING");
            detectedMotif = "NO CONNECTION";
        }

        // Display the detected motif
        telemetry.addData("---MOTIF---", "");
        telemetry.addData("Detected Motif", detectedMotif);
        telemetry.update();
    }
}
