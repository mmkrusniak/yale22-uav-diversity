package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;
import com.mmkrusniak.phys.RouteTo;

import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

public class OracleDrone extends OfflineDrone {
    public OracleDrone(Area area, int id) {
        super(area, id);
    }

    public OracleDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public List<Route> generatePlan() {
        List<Point> points = new ArrayList<>();
        for(Detectable d: area.getDetectables()) {
            points.add(new Point(d.x(), d.y(), 10.0));
        }
        DroneUtil.heuristicTSP(points, getArea().getStart(getID()), getArea().getStart(getID()));
        List<Route> result = new ArrayList<>();
        for(Point p: points) result.add(new RouteTo(p));
        return result;
    }

    @Override
    public void visualize(Graphics g) {

    }
}
