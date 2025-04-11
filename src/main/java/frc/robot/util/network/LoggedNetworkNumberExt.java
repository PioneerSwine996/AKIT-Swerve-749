package frc.robot.util.network;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.HashSet;

/** Note: automatically adds <code>/Tuning</code> to key */
public class LoggedNetworkNumberExt extends LoggedNetworkNumber {
    private final HashSet<Integer> hashCodesGottenChange = new HashSet<>();
    private double lastValue;
    private boolean hasChangedOnce = false;

    public LoggedNetworkNumberExt(String key, double defaultValue) {
        super("/Tuning/" + removeSlash(key), defaultValue);
        lastValue = defaultValue;
    }

    public boolean hasChanged(int hashCode) {
        if (hasChangedOnce && !hashCodesGottenChange.contains(hashCode)) {
            hashCodesGottenChange.add(hashCode);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        double newValue = get();
        if (newValue != lastValue) {
            // Clear all hash codes that have been given the change so that they get the new value
            hashCodesGottenChange.clear();
            hasChangedOnce = true;
        }
        lastValue = newValue;
    }
}
