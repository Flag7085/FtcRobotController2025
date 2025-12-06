package org.firstinspires.ftc.teamcode.util;

/**
 * Infinite Impulse Response filter - a form of low-pass filter that
 * is used in signal processing.  Use this to smooth out glitchy/noisy data.
 *
 * The lower the alpha value (0.0 < alpha <= 1.0), the lower the impact
 * of each new data point on the filtered output.
 *
 * For reference - given a simple step change:
 *  - An alpha of 0.5 (50%) takes 4 updates to adjust by 90% and 7 updates to
 *    adjust by 99% of the total change
 *  - An alpha of 0.6 (60%) takes 3 updates for 90% and 6 updates for 99%
 *  - An alpha of 0.7 (70%) takes 2 and 4 updates respectively
 *  - An alpha of 0.8 takes 2 and 3
 *  - An alpha of 0.9 takes 1 and 2
 *
 *  This implementation comes with a "reset threshold" which will just take
 *  the next provided data point if more than X milliseconds has passed since
 *  the last update...  This is useful when data is not always checked/available.
 */
public class IIRFilter {

    private double alpha;
    private long lastUpdate;
    private long resetThresholdMs;
    private double output;

    public IIRFilter(double alpha, long resetThresholdMs) {
        this.alpha = alpha;
        this.lastUpdate = 0;
        this.resetThresholdMs = resetThresholdMs;
        this.output = 0.0;
    }

    public double getOutput() {
        return output;
    }

    public double update(double value) {
        long now = System.currentTimeMillis();
        boolean reset = (now - lastUpdate > resetThresholdMs);
        lastUpdate = now;

        if (reset) {
            output = value;
        } else {
            output = ((1.0 - alpha) * output) + (alpha * value);
        }
        return output;
    }
}
