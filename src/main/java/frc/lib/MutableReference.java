package frc.lib;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class MutableReference<T> implements Supplier<T>, Consumer<T> {
    private T val = null;

    public MutableReference(T val) {
        set(val);
    }

    public MutableReference() {
        this(null);
    }

    @Override
    public T get() {
        return val;
    }

    @Override
    public void accept(T t) {
        val = t;
    }

    public void set(T val) {
        this.val = val;
    }
}
