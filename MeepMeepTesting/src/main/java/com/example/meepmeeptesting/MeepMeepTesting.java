package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Define the starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
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

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
