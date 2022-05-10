package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

/**
 * An offline drone that traverses a polygon in the worst way. It always traverses parallel to the girth of the
 * polygon, resulting in lots of small turns.
 */
public class UnwisePlowDrone extends OfflineDrone {

    /**
     * Initializes the drone with the default energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     */
    public UnwisePlowDrone(Area area, int id) {
        super(area, id);
    }

    /**
     * Initializes the drone with a custom energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     * @param energyBudget Energy budget of the drone.
     */
    public UnwisePlowDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    /**
     * Gives a preplanned route this drone will take (as it is an offline drone).
     * @return The route for the drone to take.
     */
    @Override
    public List<Route> generatePlan() {
        List<Route> result = new ArrayList<>(
                DroneUtil.optimizePlan(
                        DroneUtil.plow(
                                getPolygon(), getArea().getStart(getID()), DroneUtil.getCruiseAltitude(getArea()),
                                -getPolygon().base().measure())));
        result.add(0, new RouteHead(getPolygon().girth().measure()));
        return result;
    }

    /**
     * Paints a visualization of the drone on the supplied graphics instance. Good to modify for debugging; otherwise
     * does nothing to reduce visual clutter.
     * @param g A Java graphics canvas on which to paint.
     */
    @Override
    public void visualize(Graphics g) {
        // nothing special to visualize
    }
}
