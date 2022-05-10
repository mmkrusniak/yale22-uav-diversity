package com.mmkrusniak.drone;

import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Polygon;

import java.util.List;

/**
 *
 Captures contain details gathered when a simulated drone takes a picture and analyzes it. This includes the area
 covered by the picture as well as any detected objects
 */
public class Capture {
    /**
     * A list of detected objects in the image.
     */
    public List<Detectable> detectables;
    /**
     * A polygon (generally expected to be a rectangle) which describes the field of view of the picture.
     */
    public Polygon view;
    /**
     * The location from which the image was taken.
     */
    public Point location;

    public Capture(Polygon p, List<Detectable> d, Point x) {
        this.view = p;
        this.detectables = d;
        this.location = x;
    }
}
