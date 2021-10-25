package org.firstinspires.ftc.teamcode.core.thread.event.types;

/**
 * Abstract class for a event that runs a listener a single time.
 */
public abstract class RunListenerOnceEvent implements Event {
    private final Runnable listener;

    /**
     * @param listener The listener to be run.
     */
    public RunListenerOnceEvent(Runnable listener) {
        this.listener = listener;
    }

    @Override
    public boolean run() {
        listener.run();
        return true;
    }
}
