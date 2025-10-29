package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

@Autonomous(name = "Coach Dave's Test")
public class CoachDavesTestAuto extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-63, 30, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);


        Action roadRunnerTrajectory = drive.actionBuilder(beginPose)

                // Line up and shoot
                .lineToXSplineHeading(-30, Math.toRadians(135))
                .waitSeconds(1.0)

                // Go pick up first line of Artifacts
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                .lineToY(52, (pose2dDual, posePath, v) -> 10)

                // Line up and shoot
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(135)),  Math.toRadians(180))
                .waitSeconds(1.0)
                .setReversed(false)

                // Go pick up second line of Artifacts
                .setReversed(true)
                .splineTo(new Vector2d(0, 24), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(false)
                .lineToY(52, (pose2dDual, posePath, v) -> 10)

                // Line up and shoot
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(135)),  Math.toRadians(180))
                .waitSeconds(1.0)
                .setReversed(false)

                // Go park to the side near goal
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(-24, 56, Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(0.5)

                .build();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(roadRunnerTrajectory);

    }
}
