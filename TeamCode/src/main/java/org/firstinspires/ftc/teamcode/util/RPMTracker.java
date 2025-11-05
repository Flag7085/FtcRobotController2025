package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

public class RPMTracker {

    public static int MOVING_AVG_WINDOW = 4;
    public static int TRACKING_WINDOW = 10;

    public static class Point {
        public static Point of(double ts, double rpm) {
            return new Point(ts, rpm);
        }
        public double timestamp;
        public double rpm;

        public Point(double ts, double rpm) {
            this.timestamp = ts;
            this.rpm = rpm;
        }

        public Point copy() {
            return Point.of(timestamp, rpm);
        }
    }

    Deque<Point> rawData = new ArrayDeque<>();
    Deque<Point> smoothedData = new ArrayDeque<>();

    public void addPoint(double timestamp, double rpm) {
        updateRawData(Point.of(timestamp, rpm));
        updateSmoothedData(Point.of(timestamp, computeMovingAvg()));
    }

    public Point currentSmoothedRpm() {
        if (smoothedData.isEmpty()) {
            return Point.of(0, 0);
        }
        return smoothedData.peekLast().copy();
    }

    /**
     * We're looking for sharp drops in RPM.  Using smoothed RPMs,
     * find the current peak-to-trough drop.
     *
     * Return zero if the moving average is rising, otherwise
     * return the timespan and total drop of the current downtrend
     */
    public Point computeCurrentPeakToTroughDrop() {
        Point mostRecent = null;
        Point peak = null;
        Iterator<Point> points = smoothedData.descendingIterator();
        if (!points.hasNext()) {
            return Point.of(0, 0);
        } else {
            mostRecent = points.next();
            peak = mostRecent;
        }

        while (points.hasNext()) {
            Point p = points.next();
            if (p.timestamp > peak.timestamp) {
                throw new RuntimeException("RPM Points out of order");
            }
            if (p.rpm >= peak.rpm) {
                peak = p;
            } else {
                // Moving back in time, RPM stopped increasing
                // We found the peak.
                break;
            }
        }

        return Point.of(mostRecent.timestamp - peak.timestamp,
                mostRecent.rpm - peak.rpm);
    }

    private void updateRawData(Point p) {
        rawData.offer(p);
        if (rawData.size() > MOVING_AVG_WINDOW) {
            rawData.poll();
        }
    }

    private void updateSmoothedData(Point p) {
        smoothedData.offer(p);
        if (smoothedData.size() > TRACKING_WINDOW) {
            smoothedData.poll();
        }
    }

    private double computeMovingAvg() {
        double total = rawData.stream()
                .map(p -> p.rpm)
                .reduce(0.0, Double::sum);
        return  total / rawData.size();
    }

}
