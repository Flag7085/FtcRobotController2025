package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "Flywheel Testing")
//@Config
public class FlywheelTestingTeleop extends OpMode {
    private static final double SECONDS_PER_MINUTE = 60;
    public static double FLYWHEEL_SPEED = 0.5;

    public static double TICKS_PER_REVOLUTION = 28;

    DcMotorEx flywheelMotor;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "test motor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        if (gamepad1.circle) {
            flywheelMotor.setPower(FLYWHEEL_SPEED);
            telemetry.addData("Flywheel set speed: ", FLYWHEEL_SPEED);

        } else {
            flywheelMotor.setPower(0.0);
            telemetry.addData("Flywheel set speed: ", 0.0);
        }

        telemetry.addLine("Press Gamepad1 Circle to run flywheel");
        telemetry.addData("Flywheel config speed: ", FLYWHEEL_SPEED);
        telemetry.addData("Current Speed (ticks): ", flywheelMotor.getVelocity());
        telemetry.addData("Current Speed (RPM): ",
                flywheelMotor.getVelocity() * SECONDS_PER_MINUTE / TICKS_PER_REVOLUTION);

    }
}
