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
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotVersion;
import org.firstinspires.ftc.teamcode.util.IIRFilter;
import org.firstinspires.ftc.teamcode.util.RPMTracker;

@Config
public class ShooterSubsystem {
    public static DcMotorSimple.Direction SHOOTER_DIRECTION =
            Constants.ROBOT_VERSION == RobotVersion.QUALIFIERS ?
                    DcMotorSimple.Direction.FORWARD :
                    DcMotorSimple.Direction.REVERSE;
    public static double SHOOTER_TICKS_PER_REVOLUTION = 28;
    public static double TARGET_TOLERANCE = 40;

    public static double CLOSE_RANGE;
    public static double CLOSE_RPM;
    public static double FAR_RANGE;
    public static double FAR_RPM;
    public static double LONG_SHOT_RPM; // > 90 inches, i.e. across the field
    public static double LONG_SHOT_RANGE;

    static {
        switch (Constants.ROBOT_VERSION) {
            case STATES:
                CLOSE_RANGE = 24.05;
                CLOSE_RPM = 2490;
                FAR_RANGE = 57;
                FAR_RPM = 3290;
                LONG_SHOT_RANGE = 90;
                LONG_SHOT_RPM = 4010;
            case QUALIFIERS:
            default:
                CLOSE_RANGE = 49;
                CLOSE_RPM = 2850;
                FAR_RANGE = 95;
                FAR_RPM = 3350;
                LONG_SHOT_RANGE = 138;
                LONG_SHOT_RPM = 3850;
                break;
        }
    }

    PIDController pid;
    SimpleMotorFeedforward feedforward;

    double targetRPM = 0;

    DcMotorEx shooterWheel;
    DcMotorEx shooterWheel2;

    Telemetry telemetry;

    private RPMTracker rpmTracker;

    private IIRFilter lowPassFilter25 = new IIRFilter(0.25, 250);
    private IIRFilter lowPassFilter50 = new IIRFilter(0.50, 250);
    private IIRFilter lowPassFilter75 = new IIRFilter(0.75, 250);

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

         // Infinite impulse response low-pass filters for experimentation
         lowPassFilter25.update(currentRpm);
         lowPassFilter50.update(currentRpm);
         lowPassFilter75.update(currentRpm);

         double newPower = feedforward.calculate(targetRPM)
                 + pid.calculate(currentRpm, targetRPM);
         shooterWheel.setPower(newPower);
         shooterWheel2.setPower(newPower);

         rpmTracker.addPoint(System.currentTimeMillis(), currentRpm);
         double currentSmoothedRpm = rpmTracker.currentSmoothedRpm().rpm;
         RPMTracker.Point rpmDrop = rpmTracker.computeCurrentPeakToTroughDrop();

         logTelemetry("Flywheel Speed (Raw RPM)", currentRpm, p);
         logTelemetry("Flywheel Speed (Smoothed RPM)", currentSmoothedRpm, p);
         logTelemetry("Flywheel Speed (LPF 25)", lowPassFilter25.getOutput(), p);
         logTelemetry("Flywheel Speed (LPF 50)", lowPassFilter50.getOutput(), p);
         logTelemetry("Flywheel Speed (LPF 75)", lowPassFilter75.getOutput(), p);
         logTelemetry("Flywheel Drop (RPM)", rpmDrop.rpm, p);
         logTelemetry("Flywheel Drop (Rate)",
                 rpmDrop.timestamp <= 0 ? 0 : 1000 * rpmDrop.rpm / rpmDrop.timestamp, p);
         logTelemetry("Flywheel Drop Timespan (ms)", rpmDrop.timestamp, p);
         logTelemetry("Flywheel Target RPM", targetRPM, p);
         logTelemetry("Flywheel 1 current", shooterWheel.getCurrent(CurrentUnit.MILLIAMPS), p);
         logTelemetry("Flywheel 2 current", shooterWheel2.getCurrent(CurrentUnit.MILLIAMPS), p);
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
         // TODO - Tune and test - revert if needed...
         return (rangeInInches > FAR_RANGE) ?
                 interpolate(rangeInInches, FAR_RANGE, FAR_RPM, LONG_SHOT_RANGE, LONG_SHOT_RPM) :
                 interpolate(rangeInInches, CLOSE_RANGE, CLOSE_RPM, FAR_RANGE, FAR_RPM);
     }

     private double interpolate(double x, double x1, double y1, double x2, double y2) {
         double slope = (y2 - y1) / (x2 - x1); // slope = rise over run
         double intercept = y1 - slope * x1; // b = y - mx
         return slope * x + intercept; // Now interpolate: y = mx + b
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

     private void logTelemetry(String key, Object value, TelemetryPacket p) {
         if (p != null) {
             p.put(key, value);
         } else {
             telemetry.addData(key, value);
         }
     }

}
