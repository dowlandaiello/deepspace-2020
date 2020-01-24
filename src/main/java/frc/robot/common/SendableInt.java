package frc.robot.common;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;

/**
 * SendableInt represents a thread-safe, global, mutable Integer variable.
 * 
 * @author Dowland Aiello
 */
public class SendableInt {
    /**
     * A callback run after a SendableInt is changed.
     */
    public static interface Callback {
        /* The callback run once a change to the int is made */
        public void run(int changed);
    }

    /* The value of the SendableInt. */
    private int value;

    /*
     * SendableInts are thread-safe. This lock prevents threads from reading the int
     * while it is being changed.
     */
    private ReadWriteLock lock;

    /* The callback for the SendableInt. */
    private Callback callback;

    /**
     * Initializes a new SendableInt with the given value.
     * 
     * @param val the value to use in the SendableInt
     */
    public SendableInt(int val) {
        // Set the value of the mutable int
        this.value = val;
    }

    /**
     * Changes the value of the SendableInt, returning its previous value.
     */
    public int change(int val) {
        // The old value of the sendable integer
        int old = this.value;

        // Wait for the int to be writable
        Lock lock = this.lock.writeLock();

        // Set the value of the SendableInt to its previous value
        this.value = val;

        // We're done!
        lock.unlock();

        // Send back the old value
        return old;
    }

    /**
     * Gets the value of the SendableInt.
     * 
     * @return the value of the SendableInt
     */
    public int get() {
        // Return the value of the SendableInt
        return this.value;
    }

    /**
     * Registers the provided callback on the SendableInt. This callback will be run
     * once the SendableInt is changed.
     **/
    public void registerCallback(SendableInt.Callback callback) {
        // NESTED RECURSION
        Callback newCallback = (int changed) -> {
            callback.run(changed);

            this.callback.run(changed);
        };

        this.callback = newCallback;
    }
}