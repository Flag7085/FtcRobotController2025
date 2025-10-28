package org.firstinspires.ftc.teamcode.roadrunner.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class StopIntakeAction implements Action {
    IntakeSubsystem intakeSubsystem;

    public StopIntakeAction(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        intakeSubsystem.stop();
        return false;
    }
}
