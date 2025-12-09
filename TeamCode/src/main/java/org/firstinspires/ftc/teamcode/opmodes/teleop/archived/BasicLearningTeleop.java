package org.firstinspires.ftc.teamcode.opmodes.teleop.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic Learning",group = "learning")
@Disabled
public class BasicLearningTeleop extends OpMode
{

    DcMotor testMotor;
    DcMotor testMotorB;

    Servo testServo;
    
    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "test motor");
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testMotorB = hardwareMap.get(DcMotor.class, "test motor 2");
        testMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotorB.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testServo = hardwareMap.get (Servo.class, "test servo");
        testServo.setPosition(0.25);

        telemetry.addLine("Hello world!!!");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.cross) {
            testMotor.setPower(0.5);
            testMotorB.setPower(0.5);
        } else {
            testMotor.setPower(0);
            testMotorB.setPower(0);

            //testServo.setPosition(0.5);
        }
        if (gamepad1.circle) {
            testServo.setPosition(0.75);
        } else {
            testServo.setPosition(0.25);
        }
    }
}