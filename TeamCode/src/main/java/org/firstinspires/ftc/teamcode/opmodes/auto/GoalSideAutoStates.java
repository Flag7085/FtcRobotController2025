package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;

import java.util.ArrayList;
import java.util.List;

@Config
public class GoalSideAutoStates extends DecodeAuto {
    public static double SHOOTER_RPM_TARGET = 3050;

    TrajectoryActionBuilder start;
    TrajectoryActionBuilder pickUpFirstRow;
    TrajectoryActionBuilder pickUpFirstRowNoGate;
    TrajectoryActionBuilder pickUpSecondRow;
    TrajectoryActionBuilder pickUpThirdRow;
    TrajectoryActionBuilder parkOutsideLaunchZone;
    TrajectoryActionBuilder parkOutsideLaunchZoneNearGoal;
    TrajectoryActionBuilder justLeaveAction;

    // Skip the back row of artifacts, only get the front two rows
    boolean onlyDoNine = false;
    // Don't open the gate after picking up the first row
    boolean skipTheGate = false;
    // Setting this overrides everything except justLeave - if true, then no pickups
    // and robot only shoots the pre-loads
    boolean justPreloads = false;
    // Setting this overrides everything else - if true, the robot just leaves the line
    boolean justLeave = false;

    protected GoalSideAutoStates(Alliance alliance) {
        super(alliance, new Pose2d(-50, 50, Math.toRadians(126.5)));
    }

    protected GoalSideAutoStates(Alliance alliance, boolean onlyDoNine) {
        this(alliance);
        this.onlyDoNine = onlyDoNine;
    }

    protected GoalSideAutoStates(Alliance alliance, boolean onlyDoNine, boolean skipTheGate) {
        this(alliance, onlyDoNine);
        this.skipTheGate = skipTheGate;
    }

    //////////////////////////////////////////////
    //
    // Primary Autos
    //
    //////////////////////////////////////////////

    @Autonomous(name = "Goal, Red - Shoot 12 + Gate", group = "Red", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoRedAlliance extends GoalSideAutoStates {
        public GoalSideAutoRedAlliance() {
            super(Alliance.RED, false);
        }
    }

    @Autonomous(name = "Goal, Blue - Shoot 12 + Gate", group = "Blue", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoBlueAlliance extends GoalSideAutoStates {
        public GoalSideAutoBlueAlliance() {
            super(Alliance.BLUE, false);
        }
    }

    //////////////////////////////////////////////
    //
    // Auto variations - sliced and diced
    //
    //////////////////////////////////////////////

    @Autonomous(name = "Goal, Red - Shoot 9 + Gate", group = "Red", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoNineRedAlliance extends GoalSideAutoStates {
        public GoalSideAutoNineRedAlliance() {
            super(Alliance.RED, true);
        }
    }

    @Autonomous(name = "Goal, Blue - Shoot 9 + Gate", group = "Blue", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoNineBlueAlliance extends GoalSideAutoStates {
        public GoalSideAutoNineBlueAlliance() {
            super(Alliance.BLUE, true);
        }
    }

    @Autonomous(name = "Goal, Red - Shoot 9", group = "Red", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoNineNoGateRedAlliance extends GoalSideAutoStates {
        public GoalSideAutoNineNoGateRedAlliance() {
            super(Alliance.RED, true, true);
        }
    }

    @Autonomous(name = "Goal, Blue - Shoot 9 ", group = "Blue", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoNineNoGateBlueAlliance extends GoalSideAutoStates {
        public GoalSideAutoNineNoGateBlueAlliance() {
            super(Alliance.BLUE, true, true);
        }
    }

    //////////////////////////////
    //
    // Panic autos
    //
    //////////////////////////////

    @Autonomous(name = "Goal, Red - Just Leave", group = "zzzPanic", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoJustLeaveRedAlliance extends GoalSideAutoStates {
        public GoalSideAutoJustLeaveRedAlliance() {
            super(Alliance.RED);
            this.justLeave = true;
        }
    }

    @Autonomous(name = "Goal, Blue - Just Leave", group = "zzzPanic", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoJustLeaveBlueAlliance extends GoalSideAutoStates {
        public GoalSideAutoJustLeaveBlueAlliance() {
            super(Alliance.BLUE);
            this.justLeave = true;
        }
    }

    @Autonomous(name = "Goal, Red - Just Preloads", group = "zzzPanic", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoJustPreloadsRedAlliance extends GoalSideAutoStates {
        public GoalSideAutoJustPreloadsRedAlliance() {
            super(Alliance.RED);
            this.justPreloads = true;
        }
    }

    @Autonomous(name = "Goal, Blue - Just Preloads", group = "zzzPanic", preselectTeleOp = "Decode Teleop")
    public static class GoalSideAutoJustPreloadsBlueAlliance extends GoalSideAutoStates {
        public GoalSideAutoJustPreloadsBlueAlliance() {
            super(Alliance.BLUE);
            this.justPreloads = true;
        }
    }



    @Override
    public void initialize() {
        // Line up for first shot
        start = drive.actionBuilder(beginPose, poseMap())
                //.waitSeconds(10)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-25, 25), Math.toRadians(135))
                .endTrajectory();

        // Go pick up first line of Artifacts and return to shoot
        pickUpFirstRow = start.fresh()
                // Line up for intake
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                // Intake
                .afterDisp(0.0, runIntake()) // Right at the beginning
                .afterDisp(28.0, stopIntake()) // Right at the end
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

        // Go pick up first line of Artifacts and return to shoot
        pickUpFirstRowNoGate = start.fresh()
                // Line up for intake
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                // Intake
                .afterDisp(0.0, runIntake()) // Right at the beginning
                .afterDisp(28.0, stopIntake()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 15)

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
                .afterDisp(0.0, runIntake()) // Right at the beginning
                .afterDisp(28.0, stopIntake()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 15)
                // Line up and shoot
                .setReversed(true)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-25, 25, Math.toRadians(135)),  Math.toRadians(180))
                .endTrajectory();

        pickUpThirdRow = pickUpSecondRow.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d (36, 18, Math.toRadians(90)), Math.toRadians(0))

                // Intake
                .setReversed(false)
                .afterDisp(0.0, runIntake()) // Right at the beginning
                .afterDisp(28.0, stopIntake()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 15)

                // Line up and shoot
                .setReversed(true)
                .setTangent(Math.toRadians(210))
                .splineToLinearHeading(new Pose2d(-39, 19, Math.toRadians(120)), Math.toRadians(210))
                .endTrajectory();

        
        parkOutsideLaunchZone = pickUpThirdRow.fresh()
                // Go park to the side near the Gate
                .setReversed(false)
                //.setTangent(Math.toRadians(60))
                //.splineToLinearHeading(new Pose2d(-24, 56, Math.toRadians(180)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(4, 48), Math.toRadians(0))
                .endTrajectory();

        parkOutsideLaunchZoneNearGoal = start.fresh()
                .strafeToLinearHeading(new Vector2d(-30, 56), Math.toRadians(180), (pose2dDual, posePath, v) -> 20)
                .endTrajectory();

        justLeaveAction = drive.actionBuilder(beginPose, poseMap())
                .strafeToLinearHeading(new Vector2d(-30, 56), Math.toRadians(180), (pose2dDual, posePath, v) -> 20)
                .endTrajectory();
    }

    @Override
    public void runAuto() {

        List<Action> actions = new ArrayList<>();

        if (justLeave) {
            actions.add(justLeaveAction.build());
        } else if (justPreloads) {
            actions.add(shooter.setRpmAction(SHOOTER_RPM_TARGET));
            actions.add(start.build());
            actions.add(shootingActionSequence(true));
            actions.add(parkOutsideLaunchZoneNearGoal.build());
        } else {
            actions.add(shooter.setRpmAction(SHOOTER_RPM_TARGET));
            actions.add(start.build());
            actions.add(shootingActionSequence(true));

            if (skipTheGate) {
                actions.add(pickUpFirstRowNoGate.build());
            } else {
                actions.add(pickUpFirstRow.build());
            }

            actions.add(shootingActionSequence(true));
            actions.add(pickUpSecondRow.build());
            actions.add(shootingActionSequence(true));
            if (!onlyDoNine) {
                // This lands fully inside launch zone, not on the line
                // so we don't need to go park after this (and won't have time).
                actions.add(pickUpThirdRow.build());
                actions.add(shootingActionSequence(true));
            } else {
                actions.add(parkOutsideLaunchZone.build());
            }
        }

        Actions.runBlocking(
            new RaceAction(
                // This action will run forever, so the RaceAction will terminate when
                // the below sequence terminates.  Loop updates go first, so that our
                // action sequence below operates on the most up-to-date data available.
                shooter.updateForeverAction(),
                new SequentialAction(actions)
            )
        );
    }
}
