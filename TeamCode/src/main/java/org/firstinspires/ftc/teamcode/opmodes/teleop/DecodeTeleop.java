/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Set;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */

@SuppressLint("DefaultLocale")
@Config
@TeleOp(name = "Decode Teleop", group = "Robot")
public class DecodeTeleop extends OpMode {

    public static boolean UPDATE_FLYWHEEL_PID = true;

    public static boolean ENABLE_DETAILED_APRIL_TAG_TELEMETRY = false;

    public static double SHOOTER_SPEED_RPM = 3000;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    // public static double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    // public static double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    public static double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static double BEARING_THRESHOLD = 0.25; // Angled towards the tag (degrees)

    public static double DRIVE_SPEED = 0.7;
    public static double TURN_SPEED = 0.5;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    // If we don't have a specific Alliance, then we will use this set
    // to match either alliance goal for targeting.
    private static final Set<Integer> GOAL_TAGS = Set.of(
            Alliance.BLUE.getGoalAprilTagId(),
            Alliance.RED.getGoalAprilTagId()
    );

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    public static int  DECIMATION = 3;

    // LED indicatorLight
    Servo indicatorLight;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    // No IMU - We use Pinpoint, which is integrated with the MecanumDrive class.
    // This declares the IMU needed to get the current direction the robot is facing
    // IMU imu;
    MecanumDrive drive;  // Add Roadrunner drive object

    ShooterSubsystem shooterSubsystem;
    FeederSubsystem feederSubsystem;
    IntakeSubsystem intakeSubsystem;

    Alliance alliance = null;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        PIDFCoefficients c = shooterSubsystem.getPIDF();

//        if (!UPDATE_FLYWHEEL_PID) {
//            Tuning.FLYWHEEL_P = c.p;
//            Tuning.FLYWHEEL_I = c.i;
//            Tuning.FLYWHEEL_D = c.d;
//            Tuning.FLYWHEEL_F = c.f;
//        }
        UPDATE_FLYWHEEL_PID = true;
        pidTuner();

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry, shooterSubsystem, intakeSubsystem);

        indicatorLight = hardwareMap.get(Servo.class, "ledIndicator");

        telemetry.addLine("Who Can Do It??");
        telemetry.addLine("We Can Do It!!!");


        // Initialize the Apriltag Detection process
        initAprilTag();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 10);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");

        Pose2d startingPose = new Pose2d(0,0,0);
        if ((boolean)blackboard.getOrDefault(Constants.USE_POSE_FROM_AUTO, false)) {
            alliance = (Alliance) blackboard.get(Constants.ALLIANCE);
            blackboard.put(Constants.ALLIANCE, null);

            startingPose = (Pose2d)blackboard.get(Constants.POSE_FROM_AUTO);
            blackboard.put(Constants.USE_POSE_FROM_AUTO, false);
        }

        drive = new MecanumDrive(hardwareMap, startingPose);
        // imu = drive.lazyImu.get();
    }

    @Override
    public void loop() {
        pidTuner();

        PoseVelocity2d robotVelocity = drive.updatePoseEstimate();
        writeRobotPoseTelemetry(drive.localizer.getPose(), robotVelocity);

//        telemetry.addData("IMU Angle",
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        telemetry.addData("IMU Velocity",
//                imu.getRobotAngularVelocity(AngleUnit.DEGREES));

        // TODO - set and read angular velocity

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetryAprilTag(currentDetections);
        AprilTagDetection goalTag = getGoalTag(currentDetections);

        // Tell the driver what we see, and what to do.
        if (goalTag != null) {
            telemetry.addData("Found", "ID %d (%s)", goalTag.id, goalTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", goalTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", goalTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", goalTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        if (goalTag != null) {
            indicatorLight.setPosition(0.4);
        } else {
            indicatorLight.setPosition(0.0);
        }

        double driveSpeed, strafe, turn;

        double driveMultiplier = gamepad1.left_stick_button ? 1.0 : DRIVE_SPEED;

        if (gamepad1.circle && goalTag != null) {
            double headingError = -goalTag.ftcPose.bearing;

            driveSpeed = -gamepad1.left_stick_y * driveMultiplier;
            strafe = gamepad1.left_stick_x  * driveMultiplier;
            if (Math.abs(headingError) < BEARING_THRESHOLD) {
                turn = 0;
                telemetry.addData("AutoAim", "Auto: Robot aligned with AprilTag!");
            } else {
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            }
            telemetry.addData("AutoAim", "Auto: Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
        } else {
            // Manual control section
            driveSpeed = -gamepad1.left_stick_y * driveMultiplier;
            strafe = gamepad1.left_stick_x  * driveMultiplier;
            turn   = gamepad1.right_stick_x * TURN_SPEED;
            telemetry.addData("AutoAim", "Manual: Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
        }

        // Basic shooting logic
        double targetRPMs = SHOOTER_SPEED_RPM;
        if (goalTag != null) {
            targetRPMs = shooterSubsystem.calculateRPMs(goalTag.ftcPose.range);
        }
        //if (gamepad2.right_trigger > 0.5) {
        shooterSubsystem.setRPM(targetRPMs);
        //} else {
        //    shooterSubsystem.setRPM(0);
        //}
        shooterSubsystem.loop();

        if (gamepad2.cross) {
            feederSubsystem.start();
        } else {
            feederSubsystem.stop();
        }

        if (!gamepad2.cross) {
            if (gamepad1.left_trigger > 0.5) {
                intakeSubsystem.start();
            } else if (gamepad1.square) {
                intakeSubsystem.reverse();
            } else {
                intakeSubsystem.stop();
            }
        }

        telemetry.addLine("Press triangle to reset Yaw");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.triangle) {
            //imu.resetYaw();
            Pose2d currentPose = drive.localizer.getPose();
            drive.localizer.setPose(new Pose2d(0, 0, 0.0));
        }

        driveFieldRelative(driveSpeed, strafe, turn);
    }

    private void pidTuner() {
        if (UPDATE_FLYWHEEL_PID) {
            PIDFCoefficients c = new PIDFCoefficients(
                    Tuning.FLYWHEEL_P,
                    Tuning.FLYWHEEL_I,
                    Tuning.FLYWHEEL_D,
                    Tuning.FLYWHEEL_F);
            shooterSubsystem.setPIDF(c);
            UPDATE_FLYWHEEL_PID = false;
        }
        telemetry.addData("Flywheel P", Tuning.FLYWHEEL_P);
        telemetry.addData("Flywheel I", Tuning.FLYWHEEL_I);
        telemetry.addData("Flywheel D", Tuning.FLYWHEEL_D);
        telemetry.addData("Flywheel F", Tuning.FLYWHEEL_F);
    }

    private void writeRobotPoseTelemetry(Pose2d pose, PoseVelocity2d velocity) {
        telemetry.addLine(
                String.format(
                    "Robot pose: (%2.1fin, %2.1fin, %3.1fdeg)",
                    pose.position.x,
                    pose.position.y,
                    Math.toDegrees(pose.heading.toDouble())
                )
        );
        telemetry.addLine(
                String.format(
                        "Robot Velocity: (%2.1fin/sec, %2.1fin/sec, %3.1fdeg/sec)",
                        velocity.linearVel.x,
                        velocity.linearVel.y,
                        Math.toDegrees(velocity.angVel)
                )
        );
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        double headingOffset = 0;
        if (alliance == Alliance.RED) {
            headingOffset = -90;
        } else if (alliance == Alliance.BLUE) {
            headingOffset = 90;
        }

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(
                theta - drive.localizer.getPose().heading.toDouble() - headingOffset);
//        theta = AngleUnit.normalizeRadians(theta -
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        telemetry.addLine(String.format(
                "Drive Field Relative: Theta: %3.1f, \n" +
                        "Rotate: %2.1f, \n" +
                        "Forward: %2.1f xformed to %2.1f, \n" +
                        "Right: %2.1f xformed to %2.1f",
                theta, rotate, forward, newForward, right, newRight
        ));

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // Replace manual drive(...) call with Roadrunner control
        // Use setWeightedDrivePower and update for holo drive and localization
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, -right), -rotate));
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

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

    private AprilTagDetection getGoalTag(List<AprilTagDetection> detections) {
        for (AprilTagDetection detection : detections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                if (alliance != null && detection.id == alliance.getGoalAprilTagId()) {
                    return detection;
                }
                if (alliance == null && GOAL_TAGS.contains(detection.id)) {
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
