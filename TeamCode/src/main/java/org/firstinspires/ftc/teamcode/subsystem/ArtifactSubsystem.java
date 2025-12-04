package org.firstinspires.ftc.teamcode.subsystem;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArtifactSubsystem {
    NormalizedColorSensor colorSensorOne;
    NormalizedColorSensor colorSensorTwo;
    protected GobildaIndicatorLight indicatorLight;

    double maxDistance; // max distance, the farthest to the sensor an artifact can be
    double minDistance; // min distance, the closet to the sensor the artifact can be

    double colorSensorOneDistance;
    double colorSensorTwoDistance;

    public void init (HardwareMap hwMap) {
        colorSensorOne = hwMap.get (NormalizedColorSensor.class, "Proximity_1");
        colorSensorTwo = hwMap.get (NormalizedColorSensor.class, "Proximity_3");

        // proximity 1 means that there is one artifact, proximity 3 means there is three artifacts

        this.indicatorLight = new GobildaIndicatorLight(hwMap, "ledIndicator");

    }

    public boolean getDetectedColor (Telemetry telemetry) {
        NormalizedRGBA colors = colorSensorOne.getNormalizedColors(); // return 4 values

        maxDistance = 10;
        minDistance = 2;



        colorSensorOneDistance = ((DistanceSensor) colorSensorOne).getDistance(DistanceUnit.CM);
        colorSensorTwoDistance = ((DistanceSensor) colorSensorTwo).getDistance(DistanceUnit.CM);

        boolean colorSensorOneHasArtifact = colorSensorOneDistance < maxDistance && colorSensorOneDistance > minDistance;
        boolean colorSensorTwoHasArtifact = colorSensorOneDistance < maxDistance && colorSensorOneDistance > minDistance;

        if (colorSensorOneHasArtifact) {
            // if sensorOneDistance is between maxDistance and minDistance that means we have at least one artifact

            indicatorLight.setColor(GobildaIndicatorLight.Color.YELLOW);

            // what color should the light be? Yellow
        } else if (colorSensorOneHasArtifact && !colorSensorTwoHasArtifact) {
            // else if sensorOneDistance is between maxDistance and minDistance
            // and sensorTwoDistance is NOT between maxDistance and minDistance that means we have two artifacts

            indicatorLight.setColor(GobildaIndicatorLight.Color.BLUE);

            // what color should the light be? Blue
        } else if (colorSensorOneHasArtifact && colorSensorTwoHasArtifact) {
            // else if sensorOneDistance is between maxDistance and minDistance
            // and sensorTwoDistance is between maxDistance and minDistance that means we have three artifacts

            indicatorLight.setColor(GobildaIndicatorLight.Color.GREEN);

            // what color should the light be? Green
        } else {
            indicatorLight.setColor(GobildaIndicatorLight.Color.RED);
        }

        return false;
    }

}
