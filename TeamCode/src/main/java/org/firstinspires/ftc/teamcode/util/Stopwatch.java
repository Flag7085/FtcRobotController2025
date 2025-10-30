package org.firstinspires.ftc.teamcode.util;

public class Stopwatch {
    private long startTime = -1;

    public boolean isStarted() {
        return startTime > 0;
    }

    public void start() {
        startTime = System.currentTimeMillis();
    }

    public long checkTimeMs() {
        if (startTime < 0) {
            return -1;
        }
        return System.currentTimeMillis() - startTime;
    }
}
