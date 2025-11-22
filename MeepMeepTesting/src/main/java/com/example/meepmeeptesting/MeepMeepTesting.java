package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * See acmerobotics MeepMeep github page: https://github.com/acmerobotics/MeepMeep/blob/master/README.md
 */
public class MeepMeepTesting {

    public static Action demo(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(0, 0, 0))
                // DEMONSTRATION OF AVAILABLE ACTIONS

                // Drive horizontally to x=30, holding the y=0 coordinate
                .lineToX(30)

                // Turn 90 degrees to the left (counter-clockwise)
                .turn(Math.toRadians(90))

                // Drive vertically to y=30, holding the x=30 coordinate
                .lineToY(30)

                // Wait for 1 second
                .waitSeconds(1)

                // Follow a smooth curve to (x=0, y=60).
                // The 'tangent' at the end of the spline will be 180 degrees (pointing left).
                .splineTo(new Vector2d(0, 60), Math.toRadians(180))

                // Wait for 1 second
                .waitSeconds(1)

                // Follow a curve to (x=-40, y=40) while keeping the robot's heading constant at 90 degrees.
                .splineToConstantHeading(new Vector2d(-40, 40), Math.toRadians(270))

                // Follow a curve to (x=-60, y=0) while smoothly turning the robot to face 270 degrees (down).
                .splineToLinearHeading(new Pose2d(-60, 0, Math.toRadians(270)), Math.toRadians(0))

                // Drive back to the starting point (0,0)
                .lineToX(0)

        // Turn back to the original 0-degree heading
                .turn(Math.toRadians(90))
                .build();
    }


    public static Action buildMeepMeepSampleAuto(RoadRunnerBotEntity bot) {
        return bot.getDrive()
                .actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .build();
    }

    public static Action dev(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(0, 0, 0))

                .turn(Math.toRadians(360))
                // TO DO: using the bot methods documented in our demo Action,
                // develop your new test route here
                .build();
    }

    public static Action buildGoalSideAuto1(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                //.actionBuilder(new Pose2d(-63, 30, Math.toRadians(180)))
                .actionBuilder(new Pose2d(-63, 41, Math.toRadians(180)))
                // Line up and shoot
                //.lineToXSplineHeading(-30, Math.toRadians(135))
                .setReversed(true)
                .splineTo(new Vector2d(-30, 30), Math.toRadians(-45))
                .waitSeconds(1.0)



                // Go pick up first line of Artifacts
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
                .splineToSplineHeading(new Pose2d(12, 24, Math.toRadians(90)), Math.toRadians(0))
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
    }

    public static Action buildFarSideAuto1(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(62, 18, Math.toRadians(180)))
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(56, 16, Math.toRadians(155)), 0)
                .waitSeconds(1.0)

                // Pick up closest line of artifacts
                .splineTo(new Vector2d(36, 24), Math.toRadians(90))
                .lineToY(52, (pose2dDual, posePath, v) -> 10)

                // Line up to shoot
                .setReversed(true)
                .splineTo(new Vector2d(56, 16), Math.toRadians(-25))
                .waitSeconds(1.0)
                .setReversed(false)

                // Pick up artifacts from loading zone
                .splineTo(new Vector2d(48, 61), Math.toRadians(60))
                .setTangent(0)
                .lineToX(68, ((pose2dDual, posePath, v) -> 10))

                // Line up to shoot
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                .splineToSplineHeading(new Pose2d(56, 16, Math.toRadians(155)), Math.toRadians(-80))
                .waitSeconds(1.0)
                .setReversed(false)

                // Move out of launch zone
                .lineToYConstantHeading(24)
                .build();
    }

    public static Action build12ArtifactCloseAuto(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(-63, 40, 0))

                .splineTo(new Vector2d(-34, 35), Math.toRadians(145))

                .waitSeconds(4)

                .splineTo(new Vector2d(-11, 25), Math.toRadians(90))
                .lineToY(50)
                .splineTo(new Vector2d(-34, 35), Math.toRadians(145))

                .waitSeconds(4)

                .splineTo(new Vector2d(12,25), Math.toRadians(90))
                .lineToY(50)
                .splineTo(new Vector2d(-34, 35), Math.toRadians(145))

                .waitSeconds(4)

                .splineTo(new Vector2d(36, 25), Math.toRadians(90))
                .lineToY(50)
                .splineTo(new Vector2d(-34, 35), Math.toRadians(145))

                .waitSeconds(4)

                .splineTo(new Vector2d(-16, 50), Math.toRadians(180))
                // TO DO: using the bot methods documented in our demo Action,
                // develop your new test route here
                .build();
    }
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(new Pose2d(0,0,0))
                .setDimensions(13.0, 18.0)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                build12ArtifactCloseAuto(myBot)
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


}
