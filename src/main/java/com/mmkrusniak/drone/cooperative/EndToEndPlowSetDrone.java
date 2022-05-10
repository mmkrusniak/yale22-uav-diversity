package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.static_height.OfflineDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;

import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

/**
 * This class describes a coverage procedure where a number of drones split a path that a hypothetical drone with
 * energy reserves equal to the sum of the group would take. Unlike EndToEndDirectDrone, the drones in the team do
 * not descend to detected objects; this is not a live algorithm.
 *
 * The base path is a simple plow motion.
 */
public class EndToEndPlowSetDrone extends OfflineDrone {

    public EndToEndPlowSetDrone(Area area, int id) {
        super(area, id);
    }

    public EndToEndPlowSetDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public List<Route> generatePlan() {
        List<Route> result = new ArrayList<>();
        Polygon target = getPolygon();

        List<Route> totalPath = new ArrayList<>(
                DroneUtil.optimizePlan(
                        DroneUtil.plow(
                                target, getArea().getStart(getID()), DroneUtil.getCruiseAltitude(target,
                                        getTeamSize()* Drone.MAX_TRAVEL_DISTANCE),
                                -target.girth().measure())));
        totalPath.add(0, new RouteHead(target.base().measure()));
        int iStart = totalPath.size() / getTeamSize() * getID();
        for (int i = 0; i < iStart; i++) {
            // only the last heading really counts, but we'll add them all
            if(totalPath.get(i).getTarget() == null) result.add(totalPath.get(i));
        }
        for(int i = iStart; i < iStart + totalPath.size()/getTeamSize(); i++) result.add(totalPath.get(i));
        return result;
    }

    @Override
    public void visualize(Graphics g) {
    }
}
