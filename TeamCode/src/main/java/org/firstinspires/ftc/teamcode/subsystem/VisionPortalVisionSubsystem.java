package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

class VisionPortalVisionSubsystem extends VisionSubsystem {
    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    public static int DECIMATION = 3;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    VisionPortalVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        initAprilTag(hardwareMap);
    }

    @Override
    public void turnOnFtcDashboardStream(double maxFps) {
        FtcDashboard.getInstance().startCameraStream(visionPortal, maxFps);
    }

    @Override
    public GoalTag findGoalTagImpl() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection goalTag = getGoalTag(currentDetections);
        if (goalTag != null) {
            telemetry.addData("Goal Tag Found", "ID %d (%s)", goalTag.id, goalTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", goalTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", goalTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", goalTag.ftcPose.yaw);
        } else {
        }
        telemetryAprilTag(currentDetections);

        return new GoalTag() {
            @Override
            public double getHeadingOffsetDegrees() {
                return goalTag.ftcPose.bearing;
            }

            @Override
            public double getRangeInches() {
                return goalTag.ftcPose.range;
            }
        };
    }

    private AprilTagDetection getGoalTag(List<AprilTagDetection> detections) {
        for (AprilTagDetection detection : detections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                if (goalTagIds.contains(detection.id)) {
                    return detection;
                }
                // This tag is in the library, but we do not want to track it right now.
                if (ENABLE_DETAILED_APRIL_TAG_TELEMETRY) {
                    telemetry.addData("GoalTag", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                if (ENABLE_DETAILED_APRIL_TAG_TELEMETRY) {
                    telemetry.addData("GoalTag", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
        }
        return null;
    }

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(DECIMATION);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor and build the vision portal
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    private void telemetryAprilTag(List<AprilTagDetection> currentDetections) {
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if (ENABLE_DETAILED_APRIL_TAG_TELEMETRY) {
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
        }

    }   // end method telemetryAprilTag()
}
