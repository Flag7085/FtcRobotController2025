package org.firstinspires.ftc.teamcode.subsystem;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RPMTracker;
import org.junit.jupiter.api.Test;

class FeederSubsystemTest {

    @Test
    public void testShootOne_triggerV2() {

        ShooterSubsystem mockShooter = mock(ShooterSubsystem.class);
        HardwareMap mockHardwareMap = mock(HardwareMap.class);
        DcMotorEx mockFeederMotor = mock(DcMotorEx.class);
        RPMTracker mockTracker = mock(RPMTracker.class);
        TelemetryPacket mockPacket = mock(TelemetryPacket.class);

        when(mockHardwareMap.get(any(), anyString())).thenReturn(mockFeederMotor);
        when(mockShooter.getRpmTracker()).thenReturn(mockTracker);
        when(mockShooter.getRpm()).thenReturn(1000.0);
        when(mockShooter.atTargetRpm()).thenReturn(false);
        when(mockTracker.computeCurrentPeakToTroughDrop())
                .thenReturn(
                        new RPMTracker.Point(0, 0),
                        new RPMTracker.Point(5, -200),
                        new RPMTracker.Point(10, -300),
                        new RPMTracker.Point(0, 0)
                );

        FeederSubsystem feeder = new FeederSubsystem(mockHardwareMap, null, mockShooter);
        Action testAction = feeder.shootOne();

        assertTrue(testAction.run(mockPacket));
        assertTrue(testAction.run(mockPacket));
        assertTrue(testAction.run(mockPacket));
        assertEquals(0, feeder.triggerCountV2);
        assertTrue(testAction.run(mockPacket)); // Because we're still on V1
        assertEquals(1, feeder.triggerCountV2);
    }

}