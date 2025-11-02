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
                    p.heading.inverse().toDouble())),
    RED(
            new IdentityPoseMap(),
            (Pose2d p) -> p);

    // For RoadRunner trajectories
    final private PoseMap poseMap;

    // For mapping the MecanumDrive's starting pose
    final private Function<Pose2d, Pose2d> poseTransform;

    Alliance(PoseMap poseMap, Function<Pose2d, Pose2d> poseTransform) {
        this.poseMap = poseMap;
        this.poseTransform = poseTransform;
    }

    public PoseMap getPoseMap() {
        return poseMap;
    }

    public Pose2d transformPose(Pose2d p) {
        return poseTransform.apply(p);
    }
}
