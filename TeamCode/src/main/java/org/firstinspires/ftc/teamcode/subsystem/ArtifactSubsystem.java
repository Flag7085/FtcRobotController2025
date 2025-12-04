package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This subsystem detects the presence of Artifacts in the robot and sets an indicator
 * light based on current state:
 *
 * Red - no artifacts detected
 * Yellow - at least one artifact detected
 * Green - robot is full
 */
@Config
public class ArtifactSubsystem {
    public static double DEFAULT_PROXIMITY_THRESHOLD = 0.5;
    public static double PROXIMITY_ONE_THRESHOLD = DEFAULT_PROXIMITY_THRESHOLD;
    public static double PROXIMITY_FULL_THRESHOLD = DEFAULT_PROXIMITY_THRESHOLD;

    // Actually a NormalizedColorSensor.
    // Rev ColorSensorV3 objects also implement the DistanceSensor interface, which
    // is all that we care about for detecting proximity...
    private DistanceSensor colorSensorOne;
    private DistanceSensor colorSensorFull;
    private GobildaIndicatorLight indicatorLight;

    private long lastUpdatedMillis = 0;

    public void init (HardwareMap hwMap) {
        // proximity one monitors just under the feeder wheel and indicates whether at least
        // one artifact is present.

        colorSensorOne = (DistanceSensor)hwMap.get(NormalizedColorSensor.class, "proximity one");

        // proximity full monitors just inside the front of the intake and indicates whether
        // the robot is full
        colorSensorFull = (DistanceSensor)hwMap.get(NormalizedColorSensor.class, "proximity full");

        // Red    - no artifacts detected
        // Yellow - at least one artifact detected
        // Green  - robot is full
        this.indicatorLight = new GobildaIndicatorLight(hwMap, "artifact indicator");
    }

    /**
     * For teleop usage - check for artifacts and update the indicator light accordingly
     */
    public void checkForArtifacts(Telemetry telemetry) {
        long now = System.currentTimeMillis();

        // Don't update every single loop!
        // These color sensors are on the I2C bus which takes 2ms per read.  We want to keep
        // our overall loop times as low as possible.
        //
        // This should be relatively slow to change and is kind of pointless to update much
        // faster than our operators can react.
        //
        if (now - lastUpdatedMillis < 200) {
            return;
        }
        lastUpdatedMillis = now;

        // Hopefully these don't flicker too much - we may need to
        // slap on a low-pass filter to stabilize...
        boolean atLeastOneArtifact = checkProximityOne(telemetry);
        boolean robotIsFull = checkProximityFull(telemetry);

        if (robotIsFull) {
            telemetry.addData("Artifact Count", 3);
            indicatorLight.setColor(GobildaIndicatorLight.Color.GREEN);
        } else if (atLeastOneArtifact) {
            telemetry.addData("Artifact Count", 1);
            indicatorLight.setColor(GobildaIndicatorLight.Color.YELLOW);
        } else {
            telemetry.addData("Artifact Count", 0);
            indicatorLight.setColor(GobildaIndicatorLight.Color.RED);
        }
    }

    /**
     * Checks a specific sensor for the presence of an Artifact.  Don't use this
     * in teleop, use checkForArtifacts instead.
     *
     * @return true if an artifact is detected in this slot, false if not.
     */
    public boolean checkProximityOne(Telemetry telemetry) {
        double distance = colorSensorOne.getDistance(DistanceUnit.MM);
        telemetry.addData("Proximity One (mm)", distance);
        return  distance < PROXIMITY_ONE_THRESHOLD;
    }

    /**
     * Checks a specific sensor for the presence of an Artifact.  Don't use this
     * in teleop, use checkForArtifacts instead.
     *
     * @return true if an artifact is detected in this slot, false if not.
     */
    public boolean checkProximityFull(Telemetry telemetry) {
        double distance = colorSensorFull.getDistance(DistanceUnit.MM);
        telemetry.addData("Proximity Full (mm)", distance);
        return distance < PROXIMITY_FULL_THRESHOLD;
    }

}
