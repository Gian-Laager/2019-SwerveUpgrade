package ch.fridolinsrobotik.utilities;

public class Timer {
    private long lastStarted = -1;
    public Timer() {

    }

    public void start() {
        lastStarted = System.currentTimeMillis();
    }

    /**
     * @return the time when {@link #start()} has been called last. If {@link #start()} hasn't been called yet it will reutrn -1.
     */
    public long getLastStarted() {
        return lastStarted;
    }
}