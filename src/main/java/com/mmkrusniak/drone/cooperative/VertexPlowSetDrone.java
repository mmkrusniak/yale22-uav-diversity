package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.static_height.OfflineDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;
import com.mmkrusniak.phys.RouteTo;

import java.awt.Graphics;
import java.util.List;

/**
 * This class describes a cooperative coverage approach where an area is split by vertices and each 
 */
public class VertexPlowSetDrone extends OfflineDrone {

    Polygon subregion;

    public VertexPlowSetDrone(Area area, int id) {
        super(area, id);
    }

    public VertexPlowSetDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public List<Route> generatePlan() {

        subregion = getPolygon();
        int subN = subregion.numSides() / getTeamSize() + 1;
        for (int i = 0; i < getID(); i++) {
            subregion = subregion.split(0, subN)[1];
        }
        if(getID() != getTeamSize()-1) subregion = subregion.split(0, subN)[0];

        List<Route> result = DroneUtil.optimizePlan(
                        DroneUtil.plow(
                                subregion, getArea().getStart(getID()), DroneUtil.getCruiseAltitude(subregion),
                                -subregion.girth().measure()));

        result.add(0, new RouteHead(subregion.base().measure()));
        result.add(new RouteTo(area.getStart(getID())));
        return result;
    }

    @Override
    public void visualize(Graphics g) {
        subregion.render(g);
    }
}
