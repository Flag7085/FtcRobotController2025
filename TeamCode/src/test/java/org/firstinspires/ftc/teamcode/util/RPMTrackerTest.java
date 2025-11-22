package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class RPMTrackerTest {

    @Test
    public void testPeakToTrough_noDataReturnsZeros() {
        RPMTracker t = new RPMTracker();
        RPMTracker.Point p = t.computeCurrentPeakToTroughDrop();
        assertEquals(0, p.rpm);
        assertEquals(0, p.timestamp);
    }

    @Test
    public void testMovingAverageCalculations() {
        RPMTracker t = new RPMTracker();

        t.addPoint( 1, 1);

        assertEquals( 1, t.currentSmoothedRpm().rpm);
        assertEquals(1, t.currentSmoothedRpm().timestamp);

        t.addPoint(2, 2);

        assertEquals( 1.5, t.currentSmoothedRpm().rpm);
        assertEquals(2, t.currentSmoothedRpm().timestamp);

        t.addPoint(3, 3);

        assertEquals( 2, t.currentSmoothedRpm().rpm);
        assertEquals(3, t.currentSmoothedRpm().timestamp);

        t.addPoint(4, 4);

        assertEquals( 2.5, t.currentSmoothedRpm().rpm);
        assertEquals(4, t.currentSmoothedRpm().timestamp);

        t.addPoint(5, 5);

        assertEquals( 3.5, t.currentSmoothedRpm().rpm);
        assertEquals(5, t.currentSmoothedRpm().timestamp);

        assertEquals(0, t.computeCurrentPeakToTroughDrop().rpm);
        assertEquals(0, t.computeCurrentPeakToTroughDrop().timestamp);
    }

    @Test
    public void testPeakToTroughBasicFunctionality() {
        RPMTracker t = new RPMTracker();

        t.addPoint( 1, 4); // 4
        t.addPoint(2, 5);  // 4.5 // Peak
        t.addPoint(3, 4);  // 4.3333
        t.addPoint(4, 3);  // 4.0
        t.addPoint(5, 4);  // 4.0 // Trough

        assertEquals(-0.5, t.computeCurrentPeakToTroughDrop().rpm);
        assertEquals(3, t.computeCurrentPeakToTroughDrop().timestamp);
    }

    @Test
    public void testPeakToTrough_twoDrops_firstIsLarger_tracksOnlyMostRecent() {
        RPMTracker t = new RPMTracker();

        t.addPoint( 1, 10); // 10
        t.addPoint(2, 20);  // 15 // Peak
        t.addPoint(3, 15);  // 15 //
        t.addPoint(4, 10);  // 13.75
        t.addPoint(5, 5);   // 12.5
        t.addPoint( 6, 1);  // 7.75
        t.addPoint(7, 10);   // 6.5 // Trough
        t.addPoint(8, 20);   // 9
        t.addPoint(9, 1);   // 8
        t.addPoint(10, 1);   // 8

        assertEquals(-1, t.computeCurrentPeakToTroughDrop().rpm);
        assertEquals(2, t.computeCurrentPeakToTroughDrop().timestamp);
    }

    @Test
    public void testPeakToTrough_twoDrops_endsInLargest() {
        RPMTracker t = new RPMTracker();

        t.addPoint( 1, 4); // 4
        t.addPoint(2, 5); // 4.5
        t.addPoint(3, 3); // 4
        t.addPoint(4, 1); // 3.25
        t.addPoint(5, 20); // 7.25
        t.addPoint(6, 15); // 9.75
        t.addPoint(7, 10); // 11.5
        t.addPoint(8, 5); // 12.5 // Moving Avg Peak
        t.addPoint( 9, 1);  // 7.75
        t.addPoint( 10, 2);  // 4.5

        RPMTracker.Point result = t.computeCurrentPeakToTroughDrop();
        assertEquals(-8, result.rpm);
        assertEquals(2, result.timestamp);
    }

    @Test
    public void testPeakToTrough_dipExists_currentlyRising() {
        RPMTracker t = new RPMTracker();

        t.addPoint( 1, 4); // 4
        t.addPoint(2, 5); // 4.5
        t.addPoint(3, 3); // 4
        t.addPoint(4, 1); // 3.25
        t.addPoint(5, 20); // 7.25

        RPMTracker.Point result = t.computeCurrentPeakToTroughDrop();
        assertEquals(0, result.rpm);
        assertEquals(0, result.timestamp);
    }
}