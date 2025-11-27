package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.RPMTracker;

@Config
public class ShooterSubsystem {
    public static DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static double SHOOTER_TICKS_PER_REVOLUTION = 28;
    public static double TARGET_TOLERANCE = 40;

    public static double CLOSE_RANGE = 32.9;
    public static double CLOSE_RPM = 2525;
    public static double FAR_RANGE = 69.1;
    public static double FAR_RPM = 3275;

    PIDController pid;
    SimpleMotorFeedforward feedforward;

    double targetRPM = 0;

    DcMotorEx shooterWheel;
    DcMotorEx shooterWheel2;

    Telemetry telemetry;

    private RPMTracker rpmTracker;

     public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
         this.telemetry = telemetry;
         shooterWheel = hardwareMap.get(DcMotorEx.class, "shooter");
         shooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         shooterWheel.setDirection(SHOOTER_DIRECTION);
         shooterWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

         shooterWheel2 = hardwareMap.get(DcMotorEx.class, "shooter2");
         shooterWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         if (SHOOTER_DIRECTION == DcMotorSimple.Direction.FORWARD) {
             shooterWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
         } else {
             shooterWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
         }
         shooterWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

         pid = new PIDController(0, 0, 0);
         feedforward = new SimpleMotorFeedforward(0, 0);

         rpmTracker = new RPMTracker();
     }

     public PIDFCoefficients getPIDF() {
         return shooterWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
     }

     public void setPIDF(PIDFCoefficients c) {
         shooterWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
         shooterWheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
     }

     public void setCoefficients (double ks, double kv, double kp, double ki, double kd) {
         feedforward = new SimpleMotorFeedforward(ks, kv);
         pid.setPID(kp, ki, kd);
     }

     public double[] getCoifficents(){
         double[] c = new double[5];

         c[0] = feedforward.ks;
         c[1] = feedforward.kv;
         c[2] = pid.getP();
         c[3] = pid.getI();
         c[4] = pid.getD();

         return c;
     }

     public RPMTracker getRpmTracker() {
         return rpmTracker;
     }


     public void loop() {
         loop(null);
     }

     public void loop(TelemetryPacket p) {
         double currentRpm = getRpm();
         double newPower = feedforward.calculate(targetRPM)
                 + pid.calculate(currentRpm, targetRPM);
         shooterWheel.setPower(newPower);
         shooterWheel2.setPower(newPower);

         rpmTracker.addPoint(System.currentTimeMillis(), currentRpm);
         double currentSmoothedRpm = rpmTracker.currentSmoothedRpm().rpm;
         RPMTracker.Point rpmDrop = rpmTracker.computeCurrentPeakToTroughDrop();


         if (p != null) {
             p.put("Flywheel Speed (Raw RPM)", currentRpm);
             p.put("Flywheel Speed (Smoothed RPM)", currentSmoothedRpm);
             p.put("Flywheel Drop (RPM)", rpmDrop.rpm);
             p.put("Flywheel Drop (Rate)",
                     rpmDrop.timestamp <= 0 ? 0 : 1000 * rpmDrop.rpm / rpmDrop.timestamp);
             p.put("Flywheel Drop Timespan (ms)", rpmDrop.timestamp);
             p.put("Flywheel Target RPM", targetRPM);
             p.put("Flywheel 1 current", shooterWheel.getCurrent(CurrentUnit.MILLIAMPS));
             p.put("Flywheel 2 current", shooterWheel2.getCurrent(CurrentUnit.MILLIAMPS));
         } else {
             telemetry.addData("Flywheel Speed (Raw RPM)", currentRpm);
             telemetry.addData("Flywheel Speed (Smoothed RPM)", currentSmoothedRpm);
             telemetry.addData("Flywheel Drop RPM", rpmDrop.rpm);
             telemetry.addData("Flywheel Drop Rate",
                     rpmDrop.timestamp <= 0 ? 0 : 1000 * rpmDrop.rpm / rpmDrop.timestamp);
             telemetry.addData("Flywheel Drop Timespan (ms)", rpmDrop.timestamp);
             telemetry.addData("Flywheel Target RPM", targetRPM);
             telemetry.addData("Flywheel 1 current", shooterWheel.getCurrent(CurrentUnit.MILLIAMPS));
             telemetry.addData("Flywheel 2 current", shooterWheel2.getCurrent(CurrentUnit.MILLIAMPS));

         }
     }

     public boolean atTargetRpm() {
         return Math.abs(targetRPM - getRpm()) < TARGET_TOLERANCE;
     }

     public void setRPM(double rpm) {
         targetRPM = rpm;
     }

     public double getRpm() {
         return shooterWheel2.getVelocity() * 60 / SHOOTER_TICKS_PER_REVOLUTION;
     }

     public double calculateRPMs(double rangeInInches) {
         telemetry.addData("Shooter Range", rangeInInches);
         rangeInInches += 0;

         if (rangeInInches > 90) {
             return 4010;
         }

         double rangeClose = CLOSE_RANGE;
         double rangeFar = FAR_RANGE;
         double rpmClose = CLOSE_RPM;
         double rpmFar = FAR_RPM;

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

    /**
     * Performs tracking updates every "loop", running forever.
     * This is meant to be used in parallel with other actions,
     * such as the RaceAction
     */
     public Action updateForeverAction() {
         return telemetryPacket -> {
             loop(telemetryPacket);
             return true;
         };
     }

}
