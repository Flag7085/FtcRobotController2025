package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

@TeleOp(name = "States Prototype Testing")
public class StatesPrototypeTestingTeleop extends OpMode {
    IntakeSubsystem intake;
    FeederSubsystem feeder;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        feeder = new FeederSubsystem(hardwareMap, telemetry, null);
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        telemetry.addLine("Intake: cross = run, square = reverse");
        telemetry.addLine("Feeder: circle = run");

        if (gamepad1.cross) {
            intake.start();
        } else if (gamepad1.square) {
            intake.reverse();
        }
        if (gamepad1.circle) {
            boolean justTurnOn = true;
            feeder.startImplementation(justTurnOn);
        }
    }
}
