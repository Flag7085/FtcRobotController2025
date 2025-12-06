package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotVersion;
import org.firstinspires.ftc.teamcode.util.RPMTracker;
import org.firstinspires.ftc.teamcode.util.Stopwatch;
import org.firstinspires.ftc.teamcode.util.TrackingWindow;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Timer;

@Config
public class FeederSubsystem {
    public static double FEEDER_SPEED =
            Constants.ROBOT_VERSION == RobotVersion.QUALIFIERS ? 0.8 : 0.7;
    public static DcMotorSimple.Direction FEEDER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double TRIGGER_DELAY_TIME_MS = 250;
    public static double TRIGGER_RPM_THRESHOLD = -100;

    public static double TRIGGER_RPM_THRESHOLD_V2 = -100;

    long triggerCount = 0;
    long triggerCountV2 = 0;
    DcMotorEx feederWheel;
    Telemetry telemetry;

    boolean triggerLatch = false;

    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public FeederSubsystem(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            ShooterSubsystem shooterSubsystem
    ) {
        this(hardwareMap, telemetry, shooterSubsystem, null);
    }

    public FeederSubsystem(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem
    ) {
        this.feederWheel = hardwareMap.get(DcMotorEx.class, "kicker");
        feederWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederWheel.setDirection(FEEDER_DIRECTION);

        if(Constants.ROBOT_VERSION == RobotVersion.STATES){
            feederWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            feederWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        this.telemetry = telemetry;

        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public void startImplementation(boolean shouldFeed) {
        if (shouldFeed) {
            feederWheel.setPower(FEEDER_SPEED);
            if (intakeSubsystem != null) {
                intakeSubsystem.start();
            }
        } else {
            stop();
            if (intakeSubsystem != null) {
                intakeSubsystem.stop();
            }
        }
    }

    public void start() {
            startImplementation(shooterSubsystem.atTargetRpm());
    }

    public void latched_start() {
        triggerLatch |= shooterSubsystem.atTargetRpm();
        startImplementation(triggerLatch);
    }

    public void stop() {
        feederWheel.setPower(0);
        triggerLatch = false;
    }

    public Action shootOne() {
        return shootOne(false);
    }

    public Action shootOne(boolean justShoot) {
        return new Action() {
            final TrackingWindow<Double> trackingWindow = new TrackingWindow<>(6);
            final Stopwatch delayStopwatch = new Stopwatch();
            final Stopwatch runtimeStopwatch = new Stopwatch();

            boolean thresholdTriggered = false;

            private boolean shotDetected(TelemetryPacket telemetryPacket) {
                double currentRpm = shooterSubsystem.getRpm();
                trackingWindow.addMeasurement(currentRpm);

                double rpmChange = currentRpm - trackingWindow.oldestMeasurement();
                telemetryPacket.put("Flywheel RPM Change", rpmChange);

                // Look for sharp deceleration in flywheel to indicate a shot
                return rpmChange < TRIGGER_RPM_THRESHOLD;
            }

            private boolean shotStillInProgress(TelemetryPacket telemetryPacket) {
                double currentRpm = shooterSubsystem.getRpm();
                trackingWindow.addMeasurement(currentRpm);
                double rpmChange = currentRpm - trackingWindow.oldestMeasurement();
                telemetryPacket.put("Flywheel RPM Change", rpmChange);
                return rpmChange < 0;
            }

            boolean shotDetectedV2(TelemetryPacket telemetryPacket) {
                RPMTracker.Point p = shooterSubsystem.getRpmTracker()
                        .computeCurrentPeakToTroughDrop();
                if (thresholdTriggered && p.rpm >= 0) {
                    // Passed the threshold and no longer decreasing.  Reset and return triggered.
                    thresholdTriggered = false;
                    return true;
                } else if (p.rpm <= TRIGGER_RPM_THRESHOLD_V2){
                    // Track that we've passed the threshold, but for timing's sake, don't
                    // return a trigger until we've stopped decreasing, so that the next
                    // shot attempt doesn't immediately re-trigger.
                    thresholdTriggered = true;
                }
                return false;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!runtimeStopwatch.isStarted()) {
                    runtimeStopwatch.start();
                }

                // Look for sharp deceleration in flywheel to indicate a shot
                boolean shotDetected = shotDetected(telemetryPacket);
                boolean shotDetectedV2 = shotDetectedV2(telemetryPacket);

                if (shotDetectedV2) {
                    triggerCountV2++;
                }

                // TODO - refactor?  stillInProgress vs. delay?  Smoothed RPM?
                if (shotDetected && !delayStopwatch.isStarted()) {
                    triggerCount++;
                    delayStopwatch.start();
                } else if ((runtimeStopwatch.checkTimeMs() >= 1500) || (
                                delayStopwatch.isStarted() &&
                                delayStopwatch.checkTimeMs() >= TRIGGER_DELAY_TIME_MS)) {
                    stop();
                    return false;
                }
                telemetryPacket.put("Triggered", triggerCount);
                telemetryPacket.put("Triggered V2", triggerCountV2);

                if (justShoot) {
                    startImplementation(true);
                } else {
                    latched_start();
                }
                return true;
            }
        };
    }
}
