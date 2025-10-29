package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem {
    public static DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static double INTAKE_SPEED = 1.0;
    DcMotorEx intakeWheels;

    Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intake");
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeWheels.setDirection(INTAKE_DIRECTION);
        intakeWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.telemetry = telemetry;
    }

    public void start() {
        intakeWheels.setPower(INTAKE_SPEED);
    }

    public void stop() {
        intakeWheels.setPower(0);
    }

    public void reverse() { intakeWheels.setPower(-INTAKE_SPEED); }
}
