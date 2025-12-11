package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;

import java.util.ArrayList;
import java.util.List;

@Config
public class FarSideV2Auto extends DecodeAuto {

    public static double SHOOTER_RPM_TARGET = 3775;

    TrajectoryActionBuilder start;
    TrajectoryActionBuilder pickUpBackRow;
    TrajectoryActionBuilder pickUpLoadingZone;
    TrajectoryActionBuilder parkOutsideLaunchZone;
    TrajectoryActionBuilder pickUpLoadingZoneAfterGate;
    TrajectoryActionBuilder pickUpMiddleRow;
    TrajectoryActionBuilder getOutOfTheWay; // Used by justLeave to tuck as far out of the way as possible
    TrajectoryActionBuilder getOutOfTheWayAfterShot; // Used by justPreloads to tuck as far out of the way as possible
    // Some routines go back to the loading zone trying to scoop up
    // artifacts that would have been released from the gate mid-auto
    boolean skipGatePickup;

    // This adds in a second attempt at picking up released artifacts
    boolean addSecondGatePickup;

    // Normally we get loading zone, then back row
    // for 9 presets total - this skips back row
    boolean skipBackRowArtifacts;

    // Normally we get loading zone, then back row for 9 presets
    // total - this adds the middle row for 12 presents
    boolean addMiddleRowArtifacts;

    // Setting this overrides everything except justLeave - if true, then no pickups
    // and robot only shoots the pre-loads
    boolean justPreloads = false;
    // Setting this overrides everything else - if true, the robot just leaves the line
    boolean justLeave = false;

    protected FarSideV2Auto(Alliance alliance) {
        this(alliance, false, false, false, false);
    }

    protected FarSideV2Auto(
            Alliance alliance,
            boolean skipGatePickup,
            boolean skipBackRowArtifacts,
            boolean addMiddleRowArtifacts,
            boolean addSecondGatePickup
    ) {
        super(alliance, new Pose2d(62, 18, Math.toRadians(180)));

        this.skipGatePickup = skipGatePickup;
        this.skipBackRowArtifacts = skipBackRowArtifacts;
        this.addMiddleRowArtifacts = addMiddleRowArtifacts;
        this.addSecondGatePickup = addSecondGatePickup;
    }

    //////////////////////////////////////////////
    //
    // Primary Autos
    //
    //////////////////////////////////////////////

    @Autonomous(name = "Far, Red - Shoot 9 + G", group = "Red", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2AutoRedAlliance extends FarSideV2Auto {
        public FarSideV2AutoRedAlliance() {
            super(Alliance.RED);
        }
    }

    @Autonomous(name = "Far, Blue - Shoot 9 + G", group = "Blue", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2AutoBlueAlliance extends FarSideV2Auto {
        public FarSideV2AutoBlueAlliance() {
            super(Alliance.BLUE);
        }
    }

    //////////////////////////////////////////////
    //
    // Auto variations - sliced and diced
    //
    //////////////////////////////////////////////

    @Autonomous(name = "Far, Red - Shoot 9", group = "Red", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2SkipGatePickupAutoRedAlliance extends FarSideV2Auto {
        public FarSideV2SkipGatePickupAutoRedAlliance() {
            super(Alliance.RED, true, false, false, false);
        }
    }

    @Autonomous(name = "Far, Blue - Shoot 9", group = "Blue", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2SkipGatePickupAutoBlueAlliance extends FarSideV2Auto {
        public FarSideV2SkipGatePickupAutoBlueAlliance() {
            super(Alliance.BLUE, true, false, false, false);
        }
    }

    @Autonomous(name = "Far, Red - Shoot 6 + G + G", group = "Red", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2SkipBackRowAutoRedAlliance extends FarSideV2Auto {
        public FarSideV2SkipBackRowAutoRedAlliance() {
            super(Alliance.RED, false, true, false, true);
        }
    }

    @Autonomous(name = "Far, Blue - Shoot 6 + G + G", group = "Blue", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2SkipBackRowAutoBlueAlliance extends FarSideV2Auto {
        public FarSideV2SkipBackRowAutoBlueAlliance() {
            super(Alliance.BLUE, false, true, false, true);
        }
    }

    @Autonomous(name = "Far, Red - Shoot 6", group = "Red", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2JustSixAutoRedAlliance extends FarSideV2Auto {
        public FarSideV2JustSixAutoRedAlliance() {
            super(Alliance.RED, true, true, false, false);
        }
    }

    @Autonomous(name = "Far, Blue - Shoot 6", group = "Blue", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2JustSixAutoBlueAlliance extends FarSideV2Auto {
        public FarSideV2JustSixAutoBlueAlliance() {
            super(Alliance.BLUE, true, true, false, false);
        }
    }

    // TODO - Add some autos that include the Middle row


    //////////////////////////////
    //
    // Panic autos
    //
    //////////////////////////////

    @Autonomous(name = "Far, Red - Just Preloads", group = "zzzPanic", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2JustPreloadsAutoRedAlliance extends FarSideV2Auto {
        public FarSideV2JustPreloadsAutoRedAlliance() {
            super(Alliance.RED, false, false, false, false);
            this.justPreloads = true; // This overrides all other settings
        }
    }

    @Autonomous(name = "Far, Blue - Just Preloads", group = "zzzPanic", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2JustPreloadsAutoBlueAlliance extends FarSideV2Auto {
        public FarSideV2JustPreloadsAutoBlueAlliance() {
            super(Alliance.BLUE, false, false, false, false);
            this.justPreloads = true; // This overrides all other settings
        }
    }

    @Autonomous(name = "Far, Red - Just Leave", group = "zzzPanic", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2JustLeaveAutoRedAlliance extends FarSideV2Auto {
        public FarSideV2JustLeaveAutoRedAlliance() {
            super(Alliance.RED, false, false, false, false);
            this.justLeave = true; // This overrides all other settings
        }
    }

    @Autonomous(name = "Far, Blue - Just Leave", group = "zzzPanic", preselectTeleOp = "Decode Teleop")
    public static class FarSideV2JustLeaveAutoBlueAlliance extends FarSideV2Auto {
        public FarSideV2JustLeaveAutoBlueAlliance() {
            super(Alliance.BLUE, false, false, false, false);
            this.justLeave = true; // This overrides all other settings
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

        // Pick up artifacts from loading zone
        pickUpLoadingZone = pickUpBackRow.fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, 61, Math.toRadians(60)), Math.toRadians(90))
                .setTangent(0)
                // Intake
                .afterDisp(0.0, runIntake()) // Right at the beginning of next lineToX
                .afterDisp(20.0, stopIntake()) // Right at the end of next lineToX
                .lineToX(68, ((pose2dDual, posePath, v) -> 15))
                // Line up to shoot
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                // Adjusted from 160 deg... turning too far
                .splineToSplineHeading(new Pose2d(56, 16, Math.toRadians(155)), Math.toRadians(-80))
                .endTrajectory();

        // Pick up closest line of artifacts
        pickUpBackRow = start.fresh()
                .setReversed(false)
                //.splineTo(new Vector2d(36, 24), Math.toRadians(90))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(36, 24, Math.toRadians(90)), Math.toRadians(90))

                // Intake
                .afterDisp(0.0, runIntake()) // Right at the beginning
                .afterDisp(28.0, stopIntake()) // Right at the end
                .lineToY(52, (pose2dDual, posePath, v) -> 15)
                // Line up to shoot
                .setReversed(true)
                .splineTo(new Vector2d(56, 16), Math.toRadians(-20))
                .endTrajectory();

        // Assume gate was opened - go try to pick up more from the loading zone
        pickUpLoadingZoneAfterGate = pickUpBackRow.fresh()
                .setReversed(false)
                .turnTo(Math.toRadians(70))
                .splineToConstantHeading(new Vector2d(63, 30), Math.toRadians(70))
                .afterDisp(0.0, runIntake()) // Right at the beginning of next lineToX
                .afterDisp(32.0, stopIntake()) // Right at the end of next lineToX
                .splineToSplineHeading(new Pose2d(63, 61, Math.toRadians(90)), Math.toRadians(90), ((pose2dDual, posePath, v) -> 15))
                // Line up to shoot
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                // Adjusted from 160 deg... turning too far
                .splineToSplineHeading(new Pose2d(56, 16, Math.toRadians(155)), Math.toRadians(-80))
                .endTrajectory();

        pickUpMiddleRow = pickUpBackRow.fresh()
                // TODO - define this and add it into the if branch below
                .endTrajectory();

        // Move out of launch zone
        parkOutsideLaunchZone = pickUpLoadingZone.fresh()
                .setReversed(false)
                .lineToYConstantHeading(24)
                .endTrajectory();

        getOutOfTheWay = drive.actionBuilder(beginPose, poseMap())
                .strafeTo(new Vector2d(61, 38))
                .endTrajectory();

        getOutOfTheWayAfterShot = start.fresh()
                .strafeToLinearHeading(new Vector2d(61, 38), Math.toRadians(180))
                .endTrajectory();
    }

    @Override
    public void runAuto() {
        List<Action> actions = new ArrayList<>();

        if (justLeave) {
            actions.add(getOutOfTheWay.build());
        } else if (justPreloads) {
            actions.add(shooter.setRpmAction(SHOOTER_RPM_TARGET));
            actions.add(new SleepAction(0.5));
            actions.add(start.build());
            actions.add(shootingActionSequence(true));
            actions.add(getOutOfTheWayAfterShot.build());
        } else {

            actions.add(shooter.setRpmAction(SHOOTER_RPM_TARGET));
            actions.add(new SleepAction(0.5));
            actions.add(start.build());
            actions.add(shootingActionSequence(true));
            actions.add(pickUpLoadingZone.build());
            actions.add(shootingActionSequence(true));

            if (!skipBackRowArtifacts) {
                actions.add(pickUpBackRow.build());
                actions.add(shootingActionSequence(true));
            }

            if (addMiddleRowArtifacts) {
                // TODO - needs to be implemented
                // actions.add(pickUpMiddleRow.build());
                // actions.add(shootingActionSequence(true));
            }

            if (!skipGatePickup) {
                actions.add(pickUpLoadingZoneAfterGate.build());
                actions.add(shootingActionSequence(true));
            }

            if (addSecondGatePickup) {
                actions.add(pickUpLoadingZoneAfterGate.build());
                actions.add(shootingActionSequence(true));
            }

            actions.add(parkOutsideLaunchZone.build());
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
