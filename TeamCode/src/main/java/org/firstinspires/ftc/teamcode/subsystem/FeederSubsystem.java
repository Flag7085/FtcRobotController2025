package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FeederSubsystem {
    public static double FEEDER_SPEED = 0.8;
    public static DcMotorSimple.Direction FEEDER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    DcMotorEx feederWheel;
    Telemetry telemetry;

    ShooterSubsystem shooterSubsystem;

    public FeederSubsystem(HardwareMap hardwareMap, Telemetry telemetry, ShooterSubsystem shooterSubsystem) {
        this.feederWheel = hardwareMap.get(DcMotorEx.class, "kicker");
        feederWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederWheel.setDirection(FEEDER_DIRECTION);
        feederWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.telemetry = telemetry;

        this.shooterSubsystem = shooterSubsystem;
    }

    public void start() {
        if (shooterSubsystem.atTargetRpm()) {
            feederWheel.setPower(FEEDER_SPEED);
        } else {
            stop();
        }
    }

    public void stop() {
        feederWheel.setPower(0);
    }
}
