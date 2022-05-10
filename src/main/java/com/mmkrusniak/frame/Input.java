package com.mmkrusniak.frame;

import com.mmkrusniak.geom.Point;

/**
 * Quick access for common user inputs. Right now, it simply tracks the click location of the mouse within the coverage
 * panel for debugging purposes.
 */
public class Input {
    /**
     * The mouse location within the coverage panel.
     */
    public static Point mouse = new Point(0, 0);
    public static java.awt.Point mouseRaw = new java.awt.Point(0, 0);
}
