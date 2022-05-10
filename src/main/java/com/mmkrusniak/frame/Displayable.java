package com.mmkrusniak.frame;

import javax.swing.*;
import java.util.function.Consumer;
import java.util.function.Supplier;

public interface Displayable {
    JPanel toPanel(Consumer<Object> setter, Supplier<Object> getter, Object[] specs);
    boolean isBordered();
}
