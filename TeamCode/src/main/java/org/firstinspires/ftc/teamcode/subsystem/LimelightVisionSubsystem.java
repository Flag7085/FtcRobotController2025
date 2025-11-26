package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
public class LimelightVisionSubsystem extends VisionSubsystem {

    public static long CACHED_RESULT_TIMEOUT_MS = 200;
    private Limelight3A limelight;

    // Cached results to prevent rapid flickering when we get a shakey detection
    private LLResult latestGoodResult;
    private LLResultTypes.FiducialResult latestGoodFiducial;
    private long latestGoodTimestamp = 0;

    protected LimelightVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void turnOnFtcDashboardStream(double maxFps) {
        FtcDashboard.getInstance().startCameraStream(limelight, 10);
    }

    @Override
    protected GoalTag findGoalTagImpl() {
        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult goalTag = searchForGoalId(result);
        long now = System.currentTimeMillis();

        if (goalTag != null) {
            latestGoodResult = result;
            latestGoodFiducial = goalTag;
            latestGoodTimestamp = now;
            telemetry.addData("Goal Tag", "Current: %s", goalTag.getTargetPoseCameraSpace());
        } else if (now - latestGoodTimestamp < CACHED_RESULT_TIMEOUT_MS) {
            // Didn't find a good result, but we have a recent cached result so use that.
            result = latestGoodResult;
            goalTag = latestGoodFiducial;
            telemetry.addData("Goal Tag", "Cached : %s", goalTag.getTargetPoseCameraSpace());
        } else {
            telemetry.addData("Goal Tag", "Not Found");
        }
        return asGoalTag(result, goalTag);
    }

    private LLResultTypes.FiducialResult searchForGoalId(LLResult result) {
        telemetry.addData("Results", result);
        telemetry.addData("Goal IDs", goalTagIds);
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials){
                telemetry.addLine(String.format("Fiducial: %s", fiducial));
                if (goalTagIds.contains(fiducial.getFiducialId())) {
                    return fiducial;
                }
            }
        }
        return null;
    }

    private GoalTag asGoalTag(LLResult result, LLResultTypes.FiducialResult tag) {
        if (result == null || tag == null) {
            return null;
        }
        return new GoalTag() {
            @Override
            public double getHeadingOffsetDegrees() {
                // horizontal angular offset of the primary target in degrees from the crosshair
                return result.getTx();
            }

            @Override
            public double getRangeInches() {
                // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
                // Limelight Camera Space:  3d Cartesian Coordinate System with (0,0,0) at the camera lens.
                // X+ → Pointing to the right (if you were to embody the camera)
                // Y+ → Pointing downward
                // Z+ → Pointing out of the camera
                //
                // Naively assume camera pitch is 0 for consistency, so the horizontal plane is X-Z
                double distanceInMeters = Math.sqrt(
                        Math.pow( tag.getTargetPoseCameraSpace().getPosition().x, 2) +
                        Math.pow( tag.getTargetPoseCameraSpace().getPosition().z, 2));
                return distanceInMeters * 39.3700787402;
            }
        };
    }
}
