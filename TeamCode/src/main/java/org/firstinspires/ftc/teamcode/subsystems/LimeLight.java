package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimeLight {
    private Limelight3A limelight;
    public enum Motif {
        PPG,
        PGP,
        GPP,
        UNKNOWN
    }
public void init (HardwareMap hardwareMap) {
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.pipelineSwitch(0);
}
    public void start() {
        limelight.start ();
    }
    public Motif readMotif (double yaw) {
        limelight.updateRobotOrientation(yaw);
        LLResult llResult = limelight.getLatestResult();

        // Always show status first

        if (llResult != null) {

            if (llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                // Get all detected AprilTags
                java.util.List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                ;

                if (!fiducialResults.isEmpty()) {

                    int tagCount = 0;
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        tagCount++;
                        int tagId = fr.getFiducialId();

                        // Check AprilTag ID and set motif
                        if (tagId == 23) {
                            return Motif.PPG;
                        } else if (tagId == 22) {
                            return Motif.PGP;
                        } else if (tagId == 21) {
                            return Motif.GPP;
                        } else {
                            return Motif.UNKNOWN;
                        }

                    }
                }
            }
        }
        return Motif.UNKNOWN;
    }
    public TargetPosition getTargetPosition(double yaw) {
       TargetPosition targetPosition = new TargetPosition();
        limelight.updateRobotOrientation(yaw);
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            targetPosition.x = llResult.getTx();
        targetPosition.isValid = true;
        }
        return targetPosition;
    }
}
