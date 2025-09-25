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
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

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
@Config
@TeleOp(name = "Decode Teleop", group = "Robot")
public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {
    public static double SHOOTER_SPEED = 0.5;

    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

   GoBildaPinpointDriver pinpoint;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    DcMotorEx rightShooterWheel;
    DcMotorEx leftShooterWheel;

    Servo triggerServo;
    @Override
    public void init() {
        rightShooterWheel = hardwareMap.get(DcMotorEx.class, "right shooter wheel");
        rightShooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooterWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooterWheel = hardwareMap.get(DcMotorEx.class, "left shooter wheel");
        leftShooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooterWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        triggerServo = hardwareMap.get (Servo.class, "trigger");
        triggerServo.setPosition(0.25);
        telemetry.addLine("Hello world!!!");
        telemetry.update();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL Drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR Drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL Drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR Drive");

        pinpoint = getPinpoint();
//         We set the left motors in reverse which is needed for drive trains where the left
//         motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

//         This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
//         wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {

        telemetry.addLine(String.format("Motor max RPMs L: %f, R: %f",
                leftShooterWheel.getMotorType().getMaxRPM(),
                rightShooterWheel.getMotorType().getMaxRPM()));
        telemetry.addLine(String.format("Motor power: %f / %f",
                leftShooterWheel.getMotorType().getAchieveableMaxRPMFraction(),
                rightShooterWheel.getMotorType().getAchieveableMaxRPMFraction()));
        telemetry.addData("Left power", leftShooterWheel.getPower());
        telemetry.addData("Right power", rightShooterWheel.getPower());

        updatePinpoint();

        // TODO - set and read angular velocity

        // Basic driving logic
        if (gamepad2.cross) {
            rightShooterWheel.setPower(SHOOTER_SPEED);
            leftShooterWheel.setPower(SHOOTER_SPEED);
        } else {
            rightShooterWheel.setPower(0);
            leftShooterWheel.setPower(0);
            //testServo.setPosition(0.5);
        }
        if (gamepad2.circle) {
            triggerServo.setPosition(0.75);
        } else {
            triggerServo.setPosition(0.25);
        }


        telemetry.addLine("Press triangle to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.triangle) {
            imu.resetYaw();
            pinpoint.resetPosAndIMU();
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
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

    private void updatePinpoint () {
        pinpoint.update();
        if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            Pose2D pose = pinpoint.getPosition();
            telemetry.addLine(String.format(Locale.US,
                    "Pinpoint pose: (%2.1fin, %2.1fin, %3.1fdeg)",
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES)
                    ));
        } else {
            telemetry.addLine("Pinpoint not ready");
        }
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
