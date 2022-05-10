package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Polygon;

/**
 * An offline drone which splits complex polygons into simpler ones by splitting concave vertices, applying Jiao's
 * cellular decomposition algorithm. This version also combines adjacent polygons that can be simply traversed
 * together. It performs well but the decomposition is time-intensive.
 */
public class CombinatoryJiaoDrone extends JiaoDrone {
    /**
     * Initialize the drone with the default energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     */
    public CombinatoryJiaoDrone(Area area, int id) {
        super(area, id);
    }
    /**
     * Initialize the drone with a custom energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     * @param energyBudget Energy budget of the drone.
     */
    public CombinatoryJiaoDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    /**
     * Finishes off the decomposition process before planning out points.
     */
    @Override
    public void preplan() {
        recompose();
        super.preplan();
    }


    /**
     * Joins the regions created by cellular decomposition.
     */
    public void recompose() {
        for (int i = 0; i < regions.size(); i++) {
            for (int j = i; j < regions.size(); j++) {
                Polygon a = regions.get(i);
                Polygon b = regions.get(j);
                if (a == b) continue;
                if (a.sharedPoints(b).size() == 2
                        && a.girth().isParallel(b.girth(), 0.5)) {
                    regions.remove(a);
                    regions.remove(b);
                    regions.add(a.combine(b));
                    i = 0;
                    j = 0;
                }
            }
        }
    }
    @Override
    public String toString() {
        return "Drone: " + getID();
    }
}
