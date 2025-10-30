package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public abstract class DecodeAuto extends LinearOpMode {

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

    private Alliance alliance;
    protected Pose2d beginPose;
    protected MecanumDrive drive;
    protected IntakeSubsystem intake;
    protected ShooterSubsystem shooter;
    protected FeederSubsystem feeder;

    protected DecodeAuto(Alliance alliance, Pose2d beginPose) {
        this.alliance = alliance;
        this.beginPose = beginPose;
    }

    protected PoseMap poseMap() {
        return alliance.getPoseMap();
    }

    protected abstract void initialize();
    protected abstract void runAuto();

    private void initialzeCore() {
        drive = new MecanumDrive(hardwareMap, beginPose);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        feeder = new FeederSubsystem(hardwareMap, telemetry, shooter);
        initialize();
    }

    protected Action shootingActionSequence() {
        return new SequentialAction(
                feeder.shootOne(),
                intake.startIntakeAction(),
                feeder.shootOne(),
                feeder.shootOne(),
                intake.stopIntakeAction()
        );
    }

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initialzeCore();
        waitForStart();
        if (isStopRequested()) {
            return;
        }
        runAuto();
    }
}
