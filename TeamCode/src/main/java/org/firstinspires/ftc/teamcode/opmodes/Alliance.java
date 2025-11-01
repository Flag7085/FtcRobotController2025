package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2dDual;

public enum Alliance {
    BLUE(pose ->
            new Pose2dDual<>(
                    new Vector2dDual<>(
                            pose.position.x,
                            pose.position.y.unaryMinus()),
                    pose.heading.inverse())),
    RED(new IdentityPoseMap());

    final private PoseMap poseMap;

    Alliance(PoseMap poseMap) {
        this.poseMap = poseMap;
    }

    public PoseMap getPoseMap() {
        return poseMap;
    }
}
