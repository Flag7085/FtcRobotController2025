package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Tuning;
import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public abstract class DecodeAuto extends LinearOpMode {



    private final Alliance alliance;
    protected final Pose2d beginPose;
    protected final Pose2d mappedBeginPose;
    protected MecanumDrive drive;
    protected IntakeSubsystem intake;
    protected ShooterSubsystem shooter;
    protected FeederSubsystem feeder;

    protected DecodeAuto(Alliance alliance, Pose2d beginPose) {
        this.alliance = alliance;
        this.beginPose = beginPose;
        this.mappedBeginPose = alliance.transformPose(beginPose);
    }

    protected PoseMap poseMap() {
        return alliance.getPoseMap();
    }

    protected abstract void initialize();
    protected abstract void runAuto();

    private void initialzeCore() {
        blackboard.put(Constants.ALLIANCE, alliance);

        drive = new MecanumDrive(hardwareMap, mappedBeginPose);
        blackboard.put(Constants.USE_POSE_FROM_AUTO, true);
        blackboard.put(Constants.POSE_FROM_AUTO, mappedBeginPose);

        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        shooter.setPIDF(new PIDFCoefficients(
                Tuning.FLYWHEEL_P,
                Tuning.FLYWHEEL_I,
                Tuning.FLYWHEEL_D,
                Tuning.FLYWHEEL_F
        ));
        feeder = new FeederSubsystem(hardwareMap, telemetry, shooter);
        initialize();
    }

    private void initializeTelemetryKeys() {
        shooter.loop();

        // Logged by the Feeder ShootOne action...
        telemetry.addData("Triggered", 0);
        telemetry.addData("Triggered V2", 0);
        telemetry.update();
    }

    protected Action shootingActionSequence() {
        return new SequentialAction(
                feeder.shootOne(),
                intake.startIntakeAction(),
                feeder.shootOne(),
                feeder.shootOne(),
                intake.reveseIntakeAction()
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
        initializeTelemetryKeys();
        waitForStart();
        if (isStopRequested()) {
            return;
        }
        try {
            runAuto();
        } finally {
            blackboard.put(Constants.POSE_FROM_AUTO, drive.localizer.getPose());
        }

    }
}
