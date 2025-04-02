package frc.lib.util;

import java.util.function.Supplier;

// Source: https://github.com/frc6995/Robot-2025/blob/main/src/main/java/frc/robot/util/Capture.java

/**
 * "Captures" a value so that it can be modified in lambdas
 * similar to mutable reference.
 */
public class Capture<T> implements Supplier<T> {
    public T inner;

    public Capture(T inner) {
        this.inner = inner;
    }

    public T get() {return inner;}
}