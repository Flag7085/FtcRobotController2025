package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2dDual;

import java.util.function.Function;

public enum Alliance {
    BLUE(pose ->
            new Pose2dDual<>(
                    new Vector2dDual<>(
                            pose.position.x,
                            pose.position.y.unaryMinus()),
                    pose.heading.inverse()),
            (Pose2d p) -> new Pose2d(
                    p.position.x,
                    -1 * p.position.y,
                    p.heading.inverse().toDouble()),
            20
    ),
    RED(
            new IdentityPoseMap(),
            (Pose2d p) -> p,
            24
    );

    // For RoadRunner trajectories
    final private PoseMap poseMap;

    // For mapping the MecanumDrive's starting pose
    final private Function<Pose2d, Pose2d> poseTransform;

    // What ID is the april tag for this alliance's goal?
    final private int goalAprilTagId;

    Alliance(
            PoseMap poseMap,
            Function<Pose2d, Pose2d> poseTransform,
            int goalAprilTagId
    ) {
        this.poseMap = poseMap;
        this.poseTransform = poseTransform;
        this.goalAprilTagId = goalAprilTagId;
    }

    public PoseMap getPoseMap() {
        return poseMap;
    }

    public Pose2d transformPose(Pose2d p) {
        return poseTransform.apply(p);
    }

    public int getGoalAprilTagId() {
        return goalAprilTagId;
    }
}
