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
    public static int RED_TARGET_ID = 24;
    public static int BLUE_TARGET_ID = 20;
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
    public TargetPosition getTargetPosition(double yaw, int tagId) {
       TargetPosition targetPosition = new TargetPosition();
        limelight.updateRobotOrientation(yaw);
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            java.util.List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

            if (!fiducialResults.isEmpty()) {

                int tagCount = 0;
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    tagCount++;
                    int tag = fr.getFiducialId();

                    // Check AprilTag ID for the goal tag and get the tx value
                    if (tag == tagId) {
                        targetPosition.x = llResult.getTx();
                        targetPosition.isValid = true;
                    }
                }
            }

        }
        return targetPosition;
    }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the
    // "tx" value from the Limelight.
    public double limelight_aim_proportional(double txValue)
    {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .25;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. Limelight 3A hfov is 54.5
        // Normalize the txValue to [-1,0] and multiply by the kp value.
        return (txValue/27.25) * kP;
    }
}
