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
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

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

    public static boolean UPDATE_CONTROLLER_COEFFICIENTS = true;
    public static boolean LOG_DRIVE_MOTOR_POWERS = true;

    public static double SHOOTER_SPEED_RPM = 3000;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    // public static double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    // public static double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    public static double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static double BEARING_THRESHOLD = 0.25; // Angled towards the tag (degrees)

    public static double DRIVE_SPEED = 1.0;
    public static double TURN_SPEED = 0.6;

    // No IMU - We use Pinpoint, which is integrated with the MecanumDrive class.
    // This declares the IMU needed to get the current direction the robot is facing
    // IMU imu;
    MecanumDrive drive;  // Add Roadrunner drive object

    //public class TestBenchColor

    ShooterSubsystem shooterSubsystem;
    FeederSubsystem feederSubsystem;
    IntakeSubsystem intakeSubsystem;
    VisionSubsystem visionSubsystem;
    PIDController autoTurnController;

    Alliance alliance = null;

    // For tracking loop time
    long previousTime = 0;

    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public DetectedColor checkProximity(){
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        telemetry.addData("distance", distance);

        // TODO add if statements for specific colors added

        return DetectedColor.UNKNOWN;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        alliance = (Alliance) blackboard.get(Constants.ALLIANCE);

        Pose2d startingPose = new Pose2d(0,0,0);
        if ((boolean)blackboard.getOrDefault(Constants.USE_POSE_FROM_AUTO, false)) {
            startingPose = (Pose2d)blackboard.get(Constants.POSE_FROM_AUTO);
            blackboard.put(Constants.USE_POSE_FROM_AUTO, false);
        }

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        autoTurnController = new PIDController(Tuning.AIM_P, 0, Tuning.AIM_D);
        UPDATE_CONTROLLER_COEFFICIENTS = true;
        pidTuner();

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry, shooterSubsystem, intakeSubsystem);
        visionSubsystem = VisionSubsystem.createUsingLimelight(hardwareMap, telemetry);
        visionSubsystem.turnOnFtcDashboardStream(10);

        if (alliance != null) {
            visionSubsystem.setGoalTagIds(alliance.getGoalAprilTagId());
        }

        drive = new MecanumDrive(hardwareMap, startingPose);

        // Wait for the DS start button to be touched.
        telemetry.addLine("Who Can Do It??");
        telemetry.addLine("We Can Do It!!!");
        telemetry.addData(">", "Touch START to start OpMode");

        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "proximity");
    }

    @Override
    public void start() {
        visionSubsystem.start();
    }

    @Override
    public void loop() {
        logLoopTime();
        pidTuner();

        PoseVelocity2d robotVelocity = drive.updatePoseEstimate();
        telemetry.addLine("Press \"share\" to reset Yaw");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.share) {
            // TODO - adjust for driver preference.
            //
            // Since we adjust from standard field coordinates/orientation into the driver's
            // field-centric view for driving, a heading of zero here requires the reset to
            // happen while the robot is facing away from the goal towards the audience.
            //
            // We typically train drivers to reset while the robot is facing "forward"
            // from their perspective, so IF there is an alliance selected, we need to
            // subtract out the offset that will be added back in during driving
            // computations.
            //
            double newHeading = alliance == null ? 0.0 : alliance.getHeadingOffset();
            // Only update orientation, keep position unchanged...
            Pose2d currentPose = drive.localizer.getPose();
            drive.localizer.setPose(
                    new Pose2d(
                            currentPose.position.x,
                            currentPose.position.y,
                            newHeading));
        }
        writeRobotPoseTelemetry(drive.localizer.getPose(), robotVelocity);

        // This also turns on an indicator light if it finds one...
        VisionSubsystem.GoalTag goalTag = visionSubsystem.findGoalTag();

        double driveSpeed, strafe, turn;

        double driveMultiplier = gamepad1.left_stick_button ? 1.0 : DRIVE_SPEED;
        driveSpeed = -gamepad1.left_stick_y * driveMultiplier;
        strafe = gamepad1.left_stick_x  * driveMultiplier;

        if (gamepad1.triangle) {
            double heading = drive.localizer.getPose().heading.toDouble();
            double target = alliance == null ? 0.0 : alliance.getHeadingOffset();
            turn = autoTurnController.calculate(heading, target);
            telemetry.addData("AutoAim", "Base Alignment: Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
        } else if (gamepad1.circle && goalTag != null){
            double headingError = -goalTag.getHeadingOffsetDegrees();
            turn = autoTurnController.calculate(headingError, 0);
            if (Math.abs(headingError) < BEARING_THRESHOLD) {
                turn = 0;
                telemetry.addData("AutoAimAligned", "Auto: Robot aligned with AprilTag!");
            } else {
                telemetry.addData("AutoAimAligned", "Auto: Robot not aligned with AprilTag!");
            }
            telemetry.addData("AutoAim", "Auto: Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
        } else {
            turn   = gamepad1.right_stick_x * TURN_SPEED;
            telemetry.addData("AutoAimAligned", "Manual: not checked");
            telemetry.addData("AutoAim", "Manual: Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafe, turn);
            autoTurnController.calculate(0, 0);
        }

        // Basic shooting logic
        double targetRPMs = SHOOTER_SPEED_RPM;
        if (goalTag != null) {
            targetRPMs = shooterSubsystem.calculateRPMs(goalTag.getRangeInches());
        }
        shooterSubsystem.setRPM(targetRPMs);
        shooterSubsystem.loop();  // This updates PID/power, ALWAYS need to call shooterSubsystem.loop()

        if (gamepad2.cross) {
            feederSubsystem.start();
        } else if (gamepad2.triangle) {
            feederSubsystem.latched_start();
        } else {
            feederSubsystem.stop();
            // Feeder control from the manipulator overrides intake behavior
            // If those buttons are not pressed, then the driver gets control
            if (gamepad1.left_trigger > 0.5) {
                intakeSubsystem.start();
            } else if (gamepad1.square) {
                intakeSubsystem.reverse();
            } else {
                intakeSubsystem.stop();
            }
        }

        driveFieldRelative(driveSpeed, strafe, turn);
    }

    private void logLoopTime() {
        long currentTime = System.currentTimeMillis();
        if (previousTime > 0) {
            telemetry.addData("Latency (ms)", currentTime - previousTime);
        }
        previousTime = currentTime;
    }

    private void pidTuner() {
        if (UPDATE_CONTROLLER_COEFFICIENTS) {
            shooterSubsystem.setCoefficients(
                    Tuning.FLYWHEEL_S,
                    Tuning.FLYWHEEL_V,
                    Tuning.FLYWHEEL_P,
                    Tuning.FLYWHEEL_I,
                    Tuning.FLYWHEEL_D
            );

            autoTurnController.setPID(Tuning.AIM_P, 0, Tuning.AIM_D);

            UPDATE_CONTROLLER_COEFFICIENTS = false;
        }
//        telemetry.addData("Flywheel S", Tuning.FLYWHEEL_S);
//        telemetry.addData("Flywheel V", Tuning.FLYWHEEL_V);
//        telemetry.addData("Flywheel P", Tuning.FLYWHEEL_P);
//        telemetry.addData("Flywheel I", Tuning.FLYWHEEL_I);
//        telemetry.addData("Flywheel D", Tuning.FLYWHEEL_D);
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

        // If we have an alliance selected, then rotate from standard field orientation
        // into the perspective from that alliance's drive station.
        double headingOffset = alliance == null ? 0 : alliance.getHeadingOffset();

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(
                theta - drive.localizer.getPose().heading.toDouble() + headingOffset);

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

        // Log out some telemetry to sanity check
        if (LOG_DRIVE_MOTOR_POWERS) {
            drive.logDrivePowers(telemetry);
        }
    }

}
