package frc.robot.util.network;

import frc.robot.Constants;

public class LoggedTunableNumber {
    private final LoggedNetworkNumberExt inner;
    private final double defaultValue;

    public LoggedTunableNumber(String key, double defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.tuningMode) {
            inner = new LoggedNetworkNumberExt(key, defaultValue);
        } else {
            inner = null;
        }
    }

    @SuppressWarnings("DataFlowIssue") // inner is guaranteed not to be null if tuning mode is true
    public boolean hasChanged(int hashCode) {
        if (Constants.tuningMode) {
            return inner.hasChanged(hashCode);
        } else {
            return false;
        }
    }

    // FIXME: If you need these methods, you should update hasChanged to work if not in tuning mode
//    /** Updates the default value, which is used if no value in NT is found. */
//    public void setDefault(double defaultValue) {
//        if (Constants.tuningMode) {
//            inner.setDefault(defaultValue);
//        } else {
//            this.valueIfNotTuningMode = defaultValue;
//        }
//    }
//
//    /**
//     * Publishes a new value. Note that the value will not be returned by
//     * {@link #get()} until the next cycle.
//     */
//    public void set(double value) {
//        if (Constants.tuningMode) {
//            inner.set(value);
//        } else {
//            valueIfNotTuningMode = value;
//        }
//    }

    /** Returns the current value. */
    @SuppressWarnings("DataFlowIssue") // inner is guaranteed not to be null if tuning mode is true
    public double get() {
        if (Constants.tuningMode) {
            return inner.get();
        } else {
            return defaultValue;
        }
    }
}
