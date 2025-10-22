package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    private enum Alliance {
        RED,
        BLUE
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
                .turn(Math.toRadians(90))
                .build();
    }

    public static Action buildGoalSideAuto1(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(-63, 30, Math.toRadians(180)))

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
    }

    public static Action buildFarSideAuto1(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(63, 16, Math.toRadians(180)))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(56, 16, Math.toRadians(150)), 0)
                .waitSeconds(1.0)

                // Pick up closest line of artifacts
                .splineTo(new Vector2d(36, 24), Math.toRadians(90))
                .lineToY(52, (pose2dDual, posePath, v) -> 10)

                // Line up to shoot
                .setReversed(true)
                .splineTo(new Vector2d(56, 16), Math.toRadians(-30))
                .waitSeconds(1.0)
                .setReversed(false)

                // Pick up artifacts from loading zone
                .splineTo(new Vector2d(48, 60), Math.toRadians(60))
                .setTangent(0)
                .lineToX(62, ((pose2dDual, posePath, v) -> 10))

                // Line up to shoot
                .setReversed(true)
                .splineTo(new Vector2d(56, 16), Math.toRadians(-30))
                .waitSeconds(1.0)
                .setReversed(false)

                // Move out of launch zone
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(48)
                .build();
    }

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12.0, 18.0)
                .setStartPose(new Pose2d(-63, 30, Math.toRadians(180)))
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                buildFarSideAuto1(myBot)
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_PAPER)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}