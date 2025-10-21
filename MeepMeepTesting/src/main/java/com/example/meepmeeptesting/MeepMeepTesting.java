package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12.0, 18.0)
                .setStartPose(new Pose2d(-63, 30, Math.toRadians(180)))
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive()
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
//                    .splineTo(new Vector2d(30, 30), Math.PI/2)
//                    .splineToConstantHeading(new Vector2d(0, 0), -Math.PI/2)
//                    .lineToX(30)
//                    .turn(Math.toRadians(90))
//                    .lineToY(30)
//                    .turn(Math.toRadians(90))
//                    .lineToX(0)
//                    .turn(Math.toRadians(90))
//                    .lineToY(0)
//                    .turn(Math.toRadians(90))
                    .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_PAPER)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}