package org.firstinspires.ftc.teamcode.roadrunner.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystem.FeederSubsystem;

public class StopFeederAction implements Action {
    FeederSubsystem feederSubsystem;

    public StopFeederAction(FeederSubsystem feederSubsystem) {
        this.feederSubsystem = feederSubsystem;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        feederSubsystem.stop();
        return false;
    }
}
