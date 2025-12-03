package org.firstinspires.ftc.teamcode.subsystem;

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


    public void init (HardwareMap hwMap) {
        colorSensorOne = hwMap.get (NormalizedColorSensor.class, "Proximity_1");
        colorSensorTwo = hwMap.get (NormalizedColorSensor.class, "Proximity_3");

    }

    public boolean getDetectedColor (Telemetry telemetry) {
        NormalizedRGBA colors = colorSensorOne.getNormalizedColors(); // return 4 values

        ((DistanceSensor) colorSensorOne).getDistance(DistanceUnit.CM);
        ((DistanceSensor) colorSensorTwo).getDistance(DistanceUnit.CM);

        return false;
    }

}
