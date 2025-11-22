package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;

@Config
public class GoalSideAuto extends DecodeAuto {
    public static double SHOOTER_RPM_TARGET = 2800;

    TrajectoryActionBuilder start;
    TrajectoryActionBuilder pickUpFirstRow;
    TrajectoryActionBuilder pickUpSecondRow;
    TrajectoryActionBuilder parkOutsideLaunchZone;

    protected GoalSideAuto(Alliance alliance) {
        super(alliance, new Pose2d(-63, 41, Math.toRadians(180)));
    }

    @Autonomous(name = "Goal, Red - Shoot 9")
    public static class GoalSideAutoRedAlliance extends GoalSideAuto {
        public GoalSideAutoRedAlliance() {
            super(Alliance.RED);
        }
    }

    @Autonomous(name = "Goal, Blue - Shoot 9")
    public static class GoalSideAutoBlueAlliance extends GoalSideAuto {
        public GoalSideAutoBlueAlliance() {
            super(Alliance.BLUE);
        }
    }

    @Override
    public void initialize() {
        // Line up for first shot
        start = drive.actionBuilder(beginPose, poseMap())
                .setReversed(true)
                .splineTo(new Vector2d(-30, 30), Math.toRadians(-45))
                .endTrajectory();

        // Go pick up first line of Artifacts and return to shoot
        pickUpFirstRow = start.fresh()
                // Line up for intake
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 10)

                // Line up and shoot
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(135)),  Math.toRadians(180))
                .endTrajectory();

        pickUpSecondRow = pickUpFirstRow.fresh()
                // Go pick up second line of Artifacts
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(12, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 10)
                // Line up and shoot
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(135)),  Math.toRadians(180))
                .endTrajectory();

        parkOutsideLaunchZone = pickUpSecondRow.fresh()
                // Go park to the side near goal
                .setReversed(false)
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(-24, 56, Math.toRadians(180)), Math.toRadians(90))
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
                    parkOutsideLaunchZone.build()
                )
            )
        );
    }
}
