package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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

    public static Action dev(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(63, 20, Math.toRadians(180)))
                //.turnTo(Math.toRadians(180))
               .lineToX(-16)
                .turn(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(-30,30), Math.toRadians(270))
               // .turn(Math.toRadians(360))
                // TO DO: using the bot methods documented in our demo Action,
                // develop your new test route here

                .build();
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(new Pose2d(0,0,0))
                .setDimensions(12, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(dev(myBot)); // change this to dev(myBot) to run your test!

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
