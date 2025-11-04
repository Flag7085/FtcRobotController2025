package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;

public class FarSideAuto extends DecodeAuto {

    public static double SHOOTER_RPM_TARGET = 4000;

    TrajectoryActionBuilder start;
    TrajectoryActionBuilder pickUpBackRow;
    TrajectoryActionBuilder pickUpLoadingZone;
    TrajectoryActionBuilder parkOutsideLaunchZone;

    protected FarSideAuto(Alliance alliance) {
        super(alliance, new Pose2d(62, 18, Math.toRadians(180)));
    }

    @Autonomous(name = "Far, Red - Shoot 9")
    public static class FarSideAutoRedAlliance extends FarSideAuto {
        public FarSideAutoRedAlliance() {
            super(Alliance.RED);
        }
    }

    @Autonomous(name = "Far, Blue - Shoot 9")
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
                .splineToLinearHeading(new Pose2d(56, 16, Math.toRadians(158)), 0)
                .endTrajectory();

        // Pick up closest line of artifacts
        pickUpBackRow = start.fresh()
                .setReversed(false)
                .splineTo(new Vector2d(36, 24), Math.toRadians(90))
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 10)
                // Line up to shoot
                .setReversed(true)
                .splineTo(new Vector2d(56, 16), Math.toRadians(-22))
                .endTrajectory();

        // Pick up artifacts from loading zone
        pickUpLoadingZone = pickUpBackRow.fresh()
                .setReversed(false)
                .splineTo(new Vector2d(48, 61), Math.toRadians(60))
                .setTangent(0)
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning of next lineToX
                .afterDisp(20.0, intake.stopIntakeAction()) // Right at the end of next lineToX
                .lineToX(68, ((pose2dDual, posePath, v) -> 10))
                // Line up to shoot
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                .splineToSplineHeading(new Pose2d(56, 16, Math.toRadians(158)), Math.toRadians(-80))
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
                new SequentialAction(
                        shooter.setRpmAction(SHOOTER_RPM_TARGET),
                        start.build(),
                        shootingActionSequence(),
                        pickUpBackRow.build(),
                        shootingActionSequence(),
                        pickUpLoadingZone.build(),
                        shootingActionSequence(),
                        parkOutsideLaunchZone.build()
                )
        );
    }
}
