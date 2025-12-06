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
public class GoalSideAutoStates extends DecodeAuto {
    public static double SHOOTER_RPM_TARGET = 3050;

    TrajectoryActionBuilder start;
    TrajectoryActionBuilder pickUpFirstRow;
    TrajectoryActionBuilder pickUpSecondRow;
    TrajectoryActionBuilder pickUpThirdRow;
    TrajectoryActionBuilder parkOutsideLaunchZone;

    protected GoalSideAutoStates(Alliance alliance) {
        super(alliance, new Pose2d(-50, 50, Math.toRadians(126.5)));
    }

    @Autonomous(name = "Goal, Red - Shoot 9 - States")
    public static class GoalSideAutoRedAlliance extends GoalSideAutoStates {
        public GoalSideAutoRedAlliance() {
            super(Alliance.RED);
        }
    }

    @Autonomous(name = "Goal, Blue - Shoot 9 - States")
    public static class GoalSideAutoBlueAlliance extends GoalSideAutoStates {
        public GoalSideAutoBlueAlliance() {
            super(Alliance.BLUE);
        }
    }

    @Override
    public void initialize() {
        // Line up for first shot
        start = drive.actionBuilder(beginPose, poseMap())
                //.waitSeconds(10)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-25, 25, Math.toRadians(135)),Math.toRadians(-45))
                .endTrajectory();

        // Go pick up first line of Artifacts and return to shoot
        pickUpFirstRow = start.fresh()
                // Line up for intake
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 15)

                // Hit the gate!!!
                .setReversed(true)
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-4, 58, Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(0.5)

                // Line up and shoot
                .setReversed(true)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-25, 25, Math.toRadians(135)),  Math.toRadians(180))
                .endTrajectory();

        pickUpSecondRow = pickUpFirstRow.fresh()
                // Go pick up second line of Artifacts
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(14, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 15)
                // Line up and shoot
                .setReversed(true)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-25, 25, Math.toRadians(135)),  Math.toRadians(180))
                .endTrajectory();

        pickUpThirdRow = pickUpSecondRow.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d (36, 18, Math.toRadians(90)), Math.toRadians(0))
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-12, 25, Math.toRadians(0)), Math.toRadians(0))
//                .lineToX(28)
//                .splineTo(new Vector2d(36, 36), Math.toRadians(90))

                .setReversed(false)
                // Intake
                .afterDisp(0.0, intake.startIntakeAction()) // Right at the beginning
                .afterDisp(28.0, intake.stopIntakeAction()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 15)
                // Line up and shoot
                .setReversed(true)
                .setTangent(Math.toRadians(210))
                .splineToLinearHeading(new Pose2d(-39, 19, Math.toRadians(125)), Math.toRadians(180))
                .endTrajectory();

        
        parkOutsideLaunchZone = pickUpThirdRow.fresh()
                // Go park to the side near goal
//                .setReversed(false)
//                .setTangent(Math.toRadians(60))
//                .splineToLinearHeading(new Pose2d(-24, 56, Math.toRadians(180)), Math.toRadians(90))
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
                    shootingActionSequence(true),
                    pickUpFirstRow.build(),
                    shootingActionSequence(true),
                    pickUpSecondRow.build(),
                    shootingActionSequence(true),
                    pickUpThirdRow.build(),
                    shootingActionSequence(true)
                    //parkOutsideLaunchZone.build()
                )
            )
        );
    }
}
