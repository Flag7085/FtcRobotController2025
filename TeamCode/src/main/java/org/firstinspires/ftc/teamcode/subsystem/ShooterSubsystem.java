package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterSubsystem {
    public static DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static double SHOOTER_TICKS_PER_REVOLUTION = 28;
    public static double TARGET_TOLERANCE = 40;

    double targetRPM = 0;

     DcMotorEx shooterWheel;
     DcMotorEx shooterWheel2;

     Telemetry telemetry;

     public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
         this.telemetry = telemetry;
         shooterWheel = hardwareMap.get(DcMotorEx.class, "shooter");
         shooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         shooterWheel.setDirection(SHOOTER_DIRECTION);
         shooterWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

         shooterWheel2 = hardwareMap.get(DcMotorEx.class, "shooter2");
         shooterWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         if (SHOOTER_DIRECTION == DcMotorSimple.Direction.FORWARD) {
             shooterWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
         } else {
             shooterWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
         }
         shooterWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
     }

     public PIDFCoefficients getPIDF() {
         return shooterWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
     }

     public void setPIDF(PIDFCoefficients c) {
         shooterWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
         shooterWheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
     }

     public void loop() {
         telemetry.addData("Current Speed (RPM): ", getRpm());
     }

     public boolean atTargetRpm() {
         return Math.abs(targetRPM - getRpm()) < TARGET_TOLERANCE;
     }

     public void setRPM(double rpm) {
         targetRPM = rpm;
         double ticksPerSecond = rpm * SHOOTER_TICKS_PER_REVOLUTION / 60;
         shooterWheel.setVelocity(ticksPerSecond);
         shooterWheel2.setVelocity(ticksPerSecond);
     }

     public double getRpm() {
         return shooterWheel.getVelocity() * 60 / SHOOTER_TICKS_PER_REVOLUTION;
     }

     public double calculateRPMs(double rangeInInches) {
         rangeInInches += 12;

         if (rangeInInches > 90) {
             return 4010;
         }

         double rangeClose = 32.9;
         double rangeFar = 69.1;
         double rpmClose = 2525;
         double rpmFar = 3275;

         // y = mx + b
         // b = y - mx
         double slope = (rpmFar - rpmClose) / (rangeFar - rangeClose);
         double intercept = rpmClose - slope * rangeClose;

         return slope * rangeInInches + intercept;
     }

     public Action setRpmAction(double rpm) {
         return telemetryPacket -> {
             setRPM(rpm);
             return false;
         };
     }

}
