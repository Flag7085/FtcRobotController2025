package org.firstinspires.ftc.teamcode.opmodes.teleop.archived;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

@TeleOp(name = "States Prototype Testing")
@Disabled
public class StatesPrototypeTestingTeleop extends OpMode {
    public static double DRIVE_SPEED = 0.8;
    public static double TURN_SPEED = 0.6;
    public static boolean ENABLE_DRIVING = true;

    IntakeSubsystem intake;
    FeederSubsystem feeder;

    MecanumDrive drive;  // Add Roadrunner drive object

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        feeder = new FeederSubsystem(hardwareMap, telemetry, null);
        if (ENABLE_DRIVING) {
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        }
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        telemetry.addLine("Intake: cross = run, square = reverse");
        telemetry.addLine("Feeder: circle = run");
        telemetry.addLine("Driving is " + (ENABLE_DRIVING ? "ENABLED" : "DISABLED"));

        if (gamepad1.cross) {
            intake.start();
        } else if (gamepad1.square) {
            intake.reverse();
        } else {
            intake.stop();
        }

        if (gamepad1.circle) {
            boolean justTurnOn = true;
            feeder.startImplementation(justTurnOn);
        } else {
            feeder.stop();
        }

        if (ENABLE_DRIVING) {
            double driveSpeed = -gamepad1.left_stick_y * DRIVE_SPEED;
            double strafe = gamepad1.left_stick_x  * DRIVE_SPEED;
            double turn   = gamepad1.right_stick_x * TURN_SPEED;
            PoseVelocity2d robotVelocity = drive.updatePoseEstimate();
            driveFieldRelative(driveSpeed, strafe, turn);
        }
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - drive.localizer.getPose().heading.toDouble());

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(newForward, -newRight),
                        -rotate));
    }
}
