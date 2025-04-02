package frc.lib.util;

import java.util.Map;

public final class ScoringPositions {
    public enum Level {
        L1,
        L2,
        L3,
        L4
    }

    public enum Branch {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L;

        public <T> Map.Entry<Branch, T> toEntry(T value) {
            return Map.entry(this, value);
        }
    }

    private ScoringPositions() {}
}
