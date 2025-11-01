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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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
//@Config
@Disabled
@TeleOp(name = "Decode Teleop", group = "Robot")
public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {
    public static double SHOOTER_SPEED = 0.5;
    public static double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public static  double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static  double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    public static  double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static  double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static  double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    public static  double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

//    double driveSpeed = 0;        // Desired forward power/speed (-1 to +1)
//    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//    double  turn            = 0;        // Desired turning power/speed (-1 to +1)

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.

//    // This declares the four motors needed
//    DcMotor frontLeftDrive;
//    DcMotor frontRightDrive;
//    DcMotor backLeftDrive;
//    DcMotor backRightDrive;

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    public static int  DECIMATION = 3;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    DcMotorEx rightShooterWheel;
    DcMotorEx leftShooterWheel;

    Servo triggerServo;

    MecanumDrive drive;  // Add Roadrunner drive object


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        rightShooterWheel = hardwareMap.get(DcMotorEx.class, "right shooter wheel");
//        rightShooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightShooterWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightShooterWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftShooterWheel = hardwareMap.get(DcMotorEx.class, "left shooter wheel");
//        leftShooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftShooterWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftShooterWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        triggerServo = hardwareMap.get (Servo.class, "trigger");
//        triggerServo.setPosition(0.25);

        telemetry.addLine("Who Can Do It??");
        telemetry.addLine("We Can Do It!!!");

//        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL Drive");
//        frontRightDrive = hardwareMap.get(DcMotor.class, "FR Drive");
//        backLeftDrive = hardwareMap.get(DcMotor.class, "BL Drive");
//        backRightDrive = hardwareMap.get(DcMotor.class, "BR Drive");
//
////         We set the left motors in reverse which is needed for drive trains where the left
////         motors are opposite to the right ones.
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//
////         This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
////         wires, you should remove these
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the Apriltag Detection process
        initAprilTag();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 10);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        imu = drive.lazyImu.get();
    }

    @Override
    public void loop() {

//        telemetry.addLine(String.format("Motor max RPMs L: %f, R: %f", leftShooterWheel.getMotorType().getMaxRPM(),
//                rightShooterWheel.getMotorType().getMaxRPM()));
//        telemetry.addLine(String.format("Motor power: %f / %f",
//                leftShooterWheel.getMotorType().getAchieveableMaxRPMFraction(),
//                rightShooterWheel.getMotorType().getAchieveableMaxRPMFraction()));
//        telemetry.addData("Left power", leftShooterWheel.getPower());
//        telemetry.addData("Right power", rightShooterWheel.getPower());

        PoseVelocity2d robotVelocity = drive.updatePoseEstimate();
        writeRobotPoseTelemetry(drive.localizer.getPose(), robotVelocity);

        // TODO - set and read angular velocity

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetryAprilTag(currentDetections);
        AprilTagDetection goalTag = getGoalTag(currentDetections);

        // Tell the driver what we see, and what to do.
        if (goalTag != null) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", goalTag.id, goalTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", goalTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", goalTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", goalTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        double driveSpeed, strafe, turn;

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.left_bumper && goalTag != null) {
            // Determine heading, range, and yaw (tag image rotation) error.
            double rangeError = (goalTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = goalTag.ftcPose.bearing;
            double yawError = goalTag.ftcPose.yaw;

            // Define thresholds for being "aligned."
            final double RANGE_THRESHOLD = 1.0; // Close enough to the AprilTag (inches)
            final double BEARING_THRESHOLD = 5.0; // Angled towards the tag (degrees)
            final double YAW_THRESHOLD = 5.0; // Squared up to tag (degrees)

            // Check if the robot is aligned within thresholds.
            if (Math.abs(rangeError) < RANGE_THRESHOLD &&
                    Math.abs(headingError) < BEARING_THRESHOLD &&
                    Math.abs(yawError) < YAW_THRESHOLD) {
                driveSpeed = 0;
                strafe = 0;
                turn = 0;
                telemetry.addData("Auto", "Robot aligned with AprilTag!");
            } else {
                // Use speed and turn "gains" to calculate robot movement.
                driveSpeed = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
            }
        } else if (gamepad1.right_bumper && goalTag != null) {
            double headingError = -goalTag.ftcPose.bearing;
            final double BEARING_THRESHOLD = 1.0; // Angled towards the tag (degrees)
            driveSpeed = -gamepad1.left_stick_y / 2.0;
            strafe = gamepad1.left_stick_x  / 2.0;
            if (Math.abs(headingError) < BEARING_THRESHOLD) {
                turn = 0;
                telemetry.addData("Auto", "Robot aligned with AprilTag!");
            } else {
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            }
            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
        } else {
            // Manual control section
            driveSpeed = -gamepad1.left_stick_y / 2.0;
            strafe = gamepad1.left_stick_x  / 2.0;
            turn   = gamepad1.right_stick_x / 3.0;
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
        }

        // Basic shooting logic
//        if (gamepad2.cross) {
//            rightShooterWheel.setPower(SHOOTER_SPEED);
//            leftShooterWheel.setPower(SHOOTER_SPEED);
//        } else {
//            rightShooterWheel.setPower(0);
//            leftShooterWheel.setPower(0);
//            //testServo.setPosition(0.5);
//        }
//        if (gamepad2.circle) {
//            triggerServo.setPosition(0.75);
//        } else {
//            triggerServo.setPosition(0.25);
//        }


        telemetry.addLine("Press triangle to reset Yaw");
        telemetry.addLine("Hold right bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.triangle) {
            imu.resetYaw();
            Pose2d currentPose = drive.localizer.getPose();
            drive.localizer.setPose(new Pose2d(0, 0, 0.0));
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
//        if (gamepad1.right_bumper) {
//            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        } else {
        driveFieldRelative(driveSpeed, strafe, turn);
        //}
    }

    public GoBildaPinpointDriver.EncoderDirection pinpointDirectionX() {
        return GoBildaPinpointDriver.EncoderDirection.FORWARD;

    }
    public GoBildaPinpointDriver.EncoderDirection getPinpointDirectionY() {
        return GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    public GoBildaPinpointDriver getPinpoint() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-4.0, -4.0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(pinpointDirectionX(), getPinpointDirectionY());
        pinpoint.resetPosAndIMU();
        return pinpoint;
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

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(
                theta - drive.localizer.getPose().heading.toDouble());
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

//        // This calculates the power needed for each wheel based on the amount of forward,
//        // strafe right, and rotate
//        double frontLeftPower = forward + right + rotate;
//        double frontRightPower = forward - right - rotate;
//        double backRightPower = forward + right - rotate;
//        double backLeftPower = forward - right + rotate;
//
//        double maxPower = 1.0;
//        double maxSpeed = 1.0;  // make this slower for outreaches
//
//        // This is needed to make sure we don't pass > 1.0 to any wheel
//        // It allows us to keep all of the motors in proportion to what they should
//        // be and not get clipped
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//
//        // We multiply by maxSpeed so that it can be set lower for outreaches
//        // When a young child is driving the robot, we may not want to allow full
//        // speed.
//        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
//        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
//        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
//        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
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
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    return detection;
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return null;
    }

    private void telemetryAprilTag(List<AprilTagDetection> currentDetections) {
        telemetry.addData("# AprilTags Detected", currentDetections.size());

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

    }   // end method telemetryAprilTag()


}
