package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.Alliance;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public abstract class VisionSubsystem {
    public static boolean ENABLE_DETAILED_APRIL_TAG_TELEMETRY = false;
    // If we don't have a specific Alliance, then we will use this set
    // to match either alliance goal for targeting.
    private static final Set<Integer> GOAL_TAGS = Set.of(
            Alliance.BLUE.getGoalAprilTagId(),
            Alliance.RED.getGoalAprilTagId()
    );

    /**
     * This implementation uses a basic webcam paired with VisionPortal.  This assumes that the
     * webcam device is configured with the hardware name "Webcam 1"
     */
    public static VisionSubsystem createUsingVisionPortal(HardwareMap hardwareMap, Telemetry telemetry) {
        return new VisionPortalVisionSubsystem(hardwareMap, telemetry);
    }

    /**
     * This implementation uses a Limelight3A for vision tasks.  This assumes that the limelight
     * is configured with the hardware name "limelight"
     */
    public static VisionSubsystem createUsingLimelight(HardwareMap hardwareMap, Telemetry telemetry) {
        return new LimelightVisionSubsystem(hardwareMap, telemetry);
    }

    /**
     * No vision - just return null goal tag.
     */
    public static VisionSubsystem createWithNoVision(HardwareMap hardwareMap, Telemetry telemetry) {
        return new VisionSubsystem(hardwareMap, telemetry) {
            @Override public void turnOnFtcDashboardStream(double maxFps) {}
            @Override protected GoalTag findGoalTagImpl() {return null;}
        };
    }

    public interface GoalTag {
        /**
         * This returns the heading of the april tag relative to the camera/robot.
         */
        double getHeadingOffsetDegrees();

        /**
         * Distance along the horizontal plane from the camera lens to the center of the tag
         * NOTE: We started without any camera pose information, so to keep things consistent
         *       moving forward we will continue to naively assume a camera pitch of 0
         */
        double getRangeInches();
    }


    Set<Integer> goalTagIds = new HashSet<>(GOAL_TAGS);

    /**
     * LED to indicate when a tag is being tracked
     */
    protected GobildaIndicatorLight indicatorLight;

    protected Telemetry telemetry;

    protected VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.indicatorLight = new GobildaIndicatorLight(hardwareMap, "ledIndicator");
        this.telemetry = telemetry;
    }

    public void start() {}

    public abstract void turnOnFtcDashboardStream(double maxFps);

    protected abstract GoalTag findGoalTagImpl();

    public void setGoalTagIds(Integer... tagIds) {
        goalTagIds.clear();
        goalTagIds.addAll(Arrays.asList(tagIds));
    }

    public GoalTag findGoalTag() {
        GoalTag tag = findGoalTagImpl();
        setTrackingIndicatorLight(tag != null);
        return tag;
    }

    private void setTrackingIndicatorLight(boolean isTrackingGoalTag) {
        if (isTrackingGoalTag) {
            indicatorLight.setColor(GobildaIndicatorLight.Color.YELLOW_GREEN);
        } else {
            indicatorLight.setColor(GobildaIndicatorLight.Color.OFF);
        }
    }
}
