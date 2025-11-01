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
    public static double FEEDER_SPEED = 0.8;
    public static DcMotorSimple.Direction FEEDER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double TRIGGER_DELAY_TIME_MS = 250;
    public static double TRIGGER_RPM_THRESHOLD = -100;

    long triggerCount = 0;
    DcMotorEx feederWheel;
    Telemetry telemetry;

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
        feederWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.telemetry = telemetry;

        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public void start() {
        if (shooterSubsystem.atTargetRpm()) {
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

    public void stop() {
        feederWheel.setPower(0);
    }

    public Action shootOne() {
        return new Action() {
            final TrackingWindow<Double> trackingWindow = new TrackingWindow<>(6);
            final Stopwatch delayStopwatch = new Stopwatch();
            final Stopwatch runtimeStopwatch = new Stopwatch();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!runtimeStopwatch.isStarted()) {
                    runtimeStopwatch.start();
                }

                double currentRpm = shooterSubsystem.getRpm();
                trackingWindow.addMeasurement(currentRpm);

                double rpmChange = currentRpm - trackingWindow.oldestMeasurement();
                telemetryPacket.put("Flywheel RPM Change", rpmChange);
                telemetry.addData("Flywheel RPM Change", rpmChange);

                // Look for sharp deceleration in flywheel to indicate a shot
                boolean shotDetected = rpmChange < TRIGGER_RPM_THRESHOLD;

                if (shotDetected && !delayStopwatch.isStarted()) {
                    telemetry.addData("Triggered", ++triggerCount);
                    telemetryPacket.put("Triggered", triggerCount);
                    delayStopwatch.start();
                } else if ((runtimeStopwatch.checkTimeMs() >= 3000) || (
                                delayStopwatch.isStarted() &&
                                delayStopwatch.checkTimeMs() >= TRIGGER_DELAY_TIME_MS)) {
                    stop();
                    return false;
                }
                start();
                return true;
            }
        };
    }
}
