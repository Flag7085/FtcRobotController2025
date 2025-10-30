package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.Queue;

public class TrackingWindow<T> {
    Queue<T> queue;
    long windowSize;

    public TrackingWindow(long windowSize) {
        queue = new LinkedList<>();
        this.windowSize = windowSize;
    }

    public void addMeasurement(T measurement) {
        queue.offer(measurement);
        if (queue.size() > windowSize) {
            queue.poll();
        }
    }

    public T oldestMeasurement() {
        return queue.peek();
    }

}
