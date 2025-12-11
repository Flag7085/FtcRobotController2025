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
                .actionBuilder(new Pose2d(-50, 50, Math.toRadians(126.5)))
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

    public static Action buildGoalSideJustLeave(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                //.actionBuilder(new Pose2d(-63, 30, Math.toRadians(180)))
                .actionBuilder(new Pose2d(-50, 50, Math.toRadians(126.5)))
                .strafeToLinearHeading(new Vector2d(-30, 56), Math.toRadians(180), (pose2dDual, posePath, v) -> 20)
                .build();
    }

    public static Action buildGoalSideJustPreloads(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                //.actionBuilder(new Pose2d(-63, 30, Math.toRadians(180)))
                .actionBuilder(new Pose2d(-50, 50, Math.toRadians(126.5)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-25, 25), Math.toRadians(135))
                .waitSeconds(1.0)
                .strafeToLinearHeading(new Vector2d(-30, 56), Math.toRadians(180), (pose2dDual, posePath, v) -> 20)
                .build();
    }

    public static Action buildGoalSideAutoWithGate(RoadRunnerBotEntity bot) {
        return bot
                .getDrive()
                //.actionBuilder(new Pose2d(-63, 30, Math.toRadians(180)))
                .actionBuilder(new Pose2d(-50, 50, Math.toRadians(126.5)))
                // Line up and shoot
                //.lineToXSplineHeading(-30, Math.toRadians(135))
                .setReversed(true)
                .splineTo(new Vector2d(-30, 30), Math.toRadians(-45))
                .waitSeconds(1.0)



                // Go pick up first line of Artifacts
                .splineToLinearHeading(new Pose2d(-12, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                .lineToY(52, (pose2dDual, posePath, v) -> 10)
               // .lineToX()
                // Go to the gate
                .setReversed(true)
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-4, 59, Math.toRadians(180)), Math.toRadians(90))
                // Line up and shoot
                .waitSeconds(2.0)
                .setTangent(Math.toRadians(-90))
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
        // Line up to shoot
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(62, 18, Math.toRadians(180)))

                .waitSeconds(1)

                .setReversed(false)
                .setTangent(180)
                // adjusted from 160 deg... First shots too far left for some reason - adjusting...
                .splineToLinearHeading(new Pose2d(56, 16, Math.toRadians(152.5)), 0)

                .waitSeconds(3)

                .setReversed(false)
                .splineTo(new Vector2d(36, 24), Math.toRadians(90))
                // Intake
               .lineToY(52, (pose2dDual, posePath, v) -> 15)
                // Line up to shoot
                .setReversed(true)
                .splineTo(new Vector2d(56, 16), Math.toRadians(-20))

                .waitSeconds(3)

                .setReversed(false)
                .splineTo(new Vector2d(48, 61), Math.toRadians(60))
                .setTangent(0)
                // Intake
               .lineToX(68, ((pose2dDual, posePath, v) -> 15))
                // Line up to shoot
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                // Adjusted from 160 deg... turning too far
                .splineToSplineHeading(new Pose2d(56, 16, Math.toRadians(155)), Math.toRadians(-80))

                .waitSeconds(3)

                .setReversed(false)
                .lineToYConstantHeading(24)
                .build();

    }

    public static Action buildFarSideAuto2(RoadRunnerBotEntity bot) {
        // Line up to shoot
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(62, 18, Math.toRadians(180)))

                // Line up to shoot
                .setReversed(false)
                .setTangent(180)
                // adjusted from 160 deg... First shots too far left for some reason - adjusting...
                .splineToLinearHeading(new Pose2d(56, 16, Math.toRadians(152.5)), 0)
                .waitSeconds(1.5)

                // Pew pew pew
                .waitSeconds(1.2)

                // Pick up artifacts from loading zone
                .setReversed(false)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, 61, Math.toRadians(60)), Math.toRadians(90))
                .setTangent(0)
                // Intake
                .lineToX(68, ((pose2dDual, posePath, v) -> 15))
                // Line up to shoot
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                // Adjusted from 160 deg... turning too far
                .splineToSplineHeading(new Pose2d(56, 16, Math.toRadians(155)), Math.toRadians(-80))

                // Pew pew pew
                .waitSeconds(1.2)

                // Pick up Back Row
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(36, 24, Math.toRadians(90)), Math.toRadians(90))
                // Intake
                .lineToY(52, (pose2dDual, posePath, v) -> 15)
                // Line up to shoot
                .setReversed(true)
                .splineTo(new Vector2d(56, 16), Math.toRadians(-20))

                // Pew pew pew
                .waitSeconds(1.2)

                // Go get more from loading zone
                .setReversed(false)
//                .turnTo(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(65, 30), Math.toRadians(90))
                .turnTo(Math.toRadians(70))
                .splineToConstantHeading(new Vector2d(63, 30), Math.toRadians(70))
                .splineToSplineHeading(new Pose2d(63, 61, Math.toRadians(90)), Math.toRadians(90), ((pose2dDual, posePath, v) -> 15))
                // Line up to shoot
                .setReversed(true)
                .setTangent(Math.toRadians(-100))
                // Adjusted from 160 deg... turning too far
                .splineToSplineHeading(new Pose2d(56, 16, Math.toRadians(155)), Math.toRadians(-80))

                // Pew pew pew
                .waitSeconds(1.2)

                // Move out of launch zone
                .setReversed(false)
                .lineToYConstantHeading(24)

                .build();

    }

    public static Action buildFarSideJustPreloads(RoadRunnerBotEntity bot) {
        // Line up to shoot
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(62, 18, Math.toRadians(180)))

                // Line up to shoot
                .setReversed(false)
                .setTangent(180)
                // adjusted from 160 deg... First shots too far left for some reason - adjusting...
                .splineToLinearHeading(new Pose2d(56, 16, Math.toRadians(152.5)), 0)
                .waitSeconds(1.5)

                // Pew pew pew
                .waitSeconds(1.2)

                .strafeToLinearHeading(new Vector2d(61, 38), Math.toRadians(180))
                .build();

    }

    public static Action buildFarSideJustLeave(RoadRunnerBotEntity bot) {
        // Line up to shoot
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(62, 18, Math.toRadians(180)))
                .strafeTo(new Vector2d(61, 38))
                .build();

    }

    public static Action build12ArtifactCloseAuto(RoadRunnerBotEntity bot) {
        return bot
                // Pre-loaded
                .getDrive()
                .actionBuilder(new Pose2d(-50, 50, Math.toRadians(126.5)))
                .setReversed(true)
                .splineTo(new Vector2d(-34, 35), Math.toRadians(-45))

                .waitSeconds(3.5)

                // First spike mark
                .setReversed(true)
                .splineToLinearHeading(new Pose2d (-10, 19, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .splineToLinearHeading(new Pose2d(-4, 59, Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(2.0)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d (-36, 35, Math.toRadians(135)), Math.toRadians(180))

                .waitSeconds(3.5)

                // Second spike mark
                .setReversed(true)
                .splineToSplineHeading(new Pose2d (12, 20, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d (-37, 35, Math.toRadians(135)), Math.toRadians(180))

                .waitSeconds(3.5)

                // Third spike mark
                .setReversed(true)
                .splineToLinearHeading(new Pose2d (36, 18, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d (-36, 35, Math.toRadians(135)), Math.toRadians(180))

                .waitSeconds(3.5)

                // Park outside launch zone
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d (-16, 50, Math.toRadians(180)), Math.toRadians(0))
                // TO DO: using the bot methods documented in our demo Action,
                // develop your new test route here
                .build();
    }

    public static Action buildGoalSideAutoStates(RoadRunnerBotEntity bot) {
        bot.setPose(new Pose2d(-50, 50, Math.toRadians(126.5)));
        return bot
                .getDrive()
                .actionBuilder(new Pose2d(-50, 50, Math.toRadians(126.5)))
                // pre load
                //.waitSeconds(10)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-25, 25), Math.toRadians(135))

                .waitSeconds(1.5)

                // Line up for intake
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                // Intake
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

                .waitSeconds(1.5)

                // Go pick up second line of Artifacts
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(14, 24, Math.toRadians(90)), Math.toRadians(0))
                .setReversed(false)
                // Intake
                .lineToY(52, (pose2dDual, posePath, v) -> 15)
                // Line up and shoot
                .setReversed(true)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-25, 25, Math.toRadians(135)),  Math.toRadians(180))

                .waitSeconds(1.5)


                // Go pick up third row
                .setReversed(true)
                .splineToSplineHeading(new Pose2d (36, 18, Math.toRadians(90)), Math.toRadians(0))

                // Intake
                .setReversed(false)
                .lineToY(52, (pose2dDual, posePath, v) -> 15)

                // Line up and shoot
                .setReversed(true)
                .setTangent(Math.toRadians(210))
                .splineToLinearHeading(new Pose2d(-39, 19, Math.toRadians(120)), Math.toRadians(210))


                .waitSeconds(1.5)

                // Go park to the side near goal
//                .setReversed(false)
//                .setTangent(Math.toRadians(60))
//                .splineToLinearHeading(new Pose2d(-24, 56, Math.toRadians(180)), Math.toRadians(90))
                .build();
        }

        public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(new Pose2d(0,0,0))
                .setDimensions(13.0, 18.0)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(0.8 * 70, 0.8 * 120, Math.toRadians(0.8*225), Math.toRadians(0.8*2000), 11)
                .build();

        myBot.runAction(
                //buildFarSideAuto1(myBot)
                //buildFarSideAuto2(myBot)
                //buildFarSideJustLeave(myBot)
                //buildFarSideJustPreloads(myBot)
                //buildGoalSideAutoStates(myBot)
                //buildGoalSideAutoWithGate(myBot)
                //build12ArtifactCloseAuto(myBot)
                //buildGoalSideAuto1(myBot)
                //buildGoalSideJustLeave(myBot)
                buildGoalSideJustPreloads(myBot)
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
