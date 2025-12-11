package org.firstinspires.ftc.teamcode.opmodes.auto.archived;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.DecodeAuto;

//@Config
public class FarSideAuto extends DecodeAuto {

    public static double SHOOTER_RPM_TARGET = 3775;

    TrajectoryActionBuilder start;
    TrajectoryActionBuilder pickUpBackRow;
    TrajectoryActionBuilder pickUpLoadingZone;
    TrajectoryActionBuilder parkOutsideLaunchZone;

    protected FarSideAuto(Alliance alliance) {
        super(alliance, new Pose2d(62, 18, Math.toRadians(180)));
    }

    @Autonomous(name = "Far, Red - Shoot 9")
    @Disabled
    public static class FarSideAutoRedAlliance extends FarSideAuto {
        public FarSideAutoRedAlliance() {
            super(Alliance.RED);
        }
    }

    @Autonomous(name = "Far, Blue - Shoot 9")
    @Disabled
    public static class FarSideAutoBlueAlliance extends FarSideAuto {
        public FarSideAutoBlueAlliance() {
            super(Alliance.BLUE);
        }
    }

    @Override
    public void initialize() {
        // Line up for first shot
        start = drive.actionBuilder(beginPose, poseMap())
                .setReversed(false)
                .setTangent(180)
                // adjusted from 160 deg... First shots too far left for some reason - adjusting...
                .splineToLinearHeading(new Pose2d(56, 16, Math.toRadians(152.5)), 0)
                .waitSeconds(1.5)
                .endTrajectory();

        // Pick up closest line of artifacts
        pickUpBackRow = start.fresh()
                .setReversed(false)
                .splineTo(new Vector2d(36, 24), Math.toRadians(90))
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 15)
                // Line up to shoot
                .setReversed(true)
                .splineTo(new Vector2d(56, 16), Math.toRadians(-20))
                .endTrajectory();

        // Pick up artifacts from loading zone
        pickUpLoadingZone = pickUpBackRow.fresh()
                .setReversed(false)
                .splineTo(new Vector2d(48, 61), Math.toRadians(60))
                .setTangent(0)
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning of next lineToX
                .afterDisp(20.0, intake.stopIntakeAction()) // Right at the end of next lineToX
                .lineToX(68, ((pose2dDual, posePath, v) -> 15))
                // Line up to shoot
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                // Adjusted from 160 deg... turning too far
                .splineToSplineHeading(new Pose2d(56, 16, Math.toRadians(155)), Math.toRadians(-80))
                .endTrajectory();

        // Move out of launch zone
        parkOutsideLaunchZone = pickUpLoadingZone.fresh()
                .setReversed(false)
                .lineToYConstantHeading(24)
                .endTrajectory();
    }

    @Override
    public void runAuto() {
        Actions.runBlocking(
            new RaceAction(
                // This action will run forever, so the RaceAction will terminate when
                // the below sequence terminates.  Loop updates go first, so that our
                // action sequence below operates on the most up-to-date data available.
                shooter.updateForeverAction(),
                new SequentialAction(
                        shooter.setRpmAction(SHOOTER_RPM_TARGET),
                        new SleepAction(0.5),
                        start.build(),
                        shootingActionSequence(true),
                        pickUpBackRow.build(),
                        shootingActionSequence(true),
                        pickUpLoadingZone.build(),
                        shootingActionSequence(true),
                        parkOutsideLaunchZone.build()
                )
            )
        );
    }
}
