package org.firstinspires.ftc.teamcode.roadrunner.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public class RunFlywheelAction implements Action {
    private ShooterSubsystem shooterSubsystem;
    private double flywheelRpm;

    public RunFlywheelAction(ShooterSubsystem shooterSubsystem, double flywheelRpm) {
        this.shooterSubsystem = shooterSubsystem;
        this.flywheelRpm = flywheelRpm;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        shooterSubsystem.setRPM(flywheelRpm);
        return false;
    }
}
