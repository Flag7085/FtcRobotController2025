package org.firstinspires.ftc.teamcode.opmodes.auto.archived;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.DecodeAuto;

//@Config
public class ShootTwelveArtifactCloseAuto extends DecodeAuto {
    public static double SHOOTER_RPM_TARGET = 2800;

    TrajectoryActionBuilder start;
    TrajectoryActionBuilder pickUpFirstRow;
    TrajectoryActionBuilder pickUpSecondRow;
    TrajectoryActionBuilder pickUpThirdRow;
    TrajectoryActionBuilder parkOutsideLaunchZone;

    protected ShootTwelveArtifactCloseAuto(Alliance alliance) {
        super(alliance, (new Pose2d(-50, 50, Math.toRadians(126.5))));
    }

    @Autonomous(name = "Close, Red - Shoot 12")
    @Disabled
    public static class ShootTwelveArtifactCloseAutoRedAlliance extends ShootTwelveArtifactCloseAuto {
        public ShootTwelveArtifactCloseAutoRedAlliance() {
            super(Alliance.RED);
        }
    }

    @Autonomous(name = "Close, Blue - Shoot 12")
    @Disabled
    public static class ShootTwelveArtifactCloseAutoBlueAlliance extends ShootTwelveArtifactCloseAuto {
        public ShootTwelveArtifactCloseAutoBlueAlliance() {
            super(Alliance.BLUE);
        }
    }

    @Override
    public void initialize() {
        // Line up for first shot
        start = drive.actionBuilder(beginPose, poseMap())
                .setReversed(true)
                .splineTo(new Vector2d(-34, 35), Math.toRadians(-45))
                .endTrajectory();

        // Go pick up first line of Artifacts and return to shoot
        pickUpFirstRow = start.fresh()
                // Line up for intake
                .setReversed(true)
                .splineToLinearHeading(new Pose2d (-10, 19, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(50, (pose2dDual, posePath, v) -> 10)
                .splineToLinearHeading(new Pose2d(-4, 59, Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(2.0)
                // Line up and shoot
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d (-36, 35, Math.toRadians(135)), Math.toRadians(180))
                .endTrajectory();

        pickUpSecondRow = pickUpFirstRow.fresh()
                // Go pick up second line of Artifacts
                .setReversed(true)
                .splineToSplineHeading(new Pose2d (12, 20, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(50, (pose2dDual, posePath, v) -> 10)
                // Line up and shoot
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d (-37, 35, Math.toRadians(135)), Math.toRadians(180))
                .endTrajectory();

        pickUpThirdRow = pickUpSecondRow.fresh()
                // Go pick up third line of Artifacts
                .setReversed(true)
                .splineToLinearHeading(new Pose2d (36, 18, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) //Right at the end
                .lineToY(50, (pose2dDual, posePath, v) -> 10)
                // Line up and shoot
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d (-36, 35, Math.toRadians(135)), Math.toRadians(180))
                .endTrajectory();

        parkOutsideLaunchZone = pickUpThirdRow.fresh()
                // Go park to the side near goal
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d (-16, 50, Math.toRadians(180)), Math.toRadians(0))
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
                            start.build(),
                            shootingActionSequence(),
                            pickUpFirstRow.build(),
                            shootingActionSequence(),
                            pickUpSecondRow.build(),
                            shootingActionSequence(),
                            pickUpThirdRow.build(),
                            shootingActionSequence(),
                            parkOutsideLaunchZone.build()
                    )
                )
        );
    }
}
