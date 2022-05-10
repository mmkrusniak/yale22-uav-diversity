package com.mmkrusniak.drone.variable_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.geom.Util;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.drone.static_height.JiaoDrone;
import com.mmkrusniak.phys.Route;

import java.util.*;

import static com.mmkrusniak.drone.DroneUtil.*;

public class ImprovedDirectDrone extends DirectDrone {

    public ImprovedDirectDrone(Area area, int id) {
        super(area, id);
    }

    public ImprovedDirectDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    public void onBegin() {
        super.onBegin();
        JiaoDrone basis = new JiaoDrone(area, getID());

        // This is a variable height drone; we'll be basing things off of measurements we take at a high altitude
        LinkedList<Route> overviewPoints = new LinkedList<>(basis.generatePlan());

        // We'd like to skirt the polygon on our way back
        Polygon poly = getPolygon();
        List<Point> skirt = getPolygon().verticesBetween(
                poly.closest(overviewPoints.get(overviewPoints.size()-1).getTarget()),
                poly.closest(area.getStart(getID())));
        List<Route> toAdd = new ArrayList<>();
        for(Line l: Line.arrayFromPoints(skirt.toArray(new Point[0]))) {
            for (Route r : overviewPoints) {
                if (r.getTarget() == null) continue;
                Point target = r.getTarget();
                if (Util.within(l.a().x(), l.b().x(), target.x())) {
                    double extent = 1.0 - (Math.abs(target.x() - l.midpoint().x()))/l.dx()*2;
                    r.setTarget(new Point(target.x(), target.y() + 30*extent, target.z()));
                }
            }
            toAdd.add(new RouteHead(l.measure()));
            toAdd.addAll(optimizePlan(subdivide(Arrays.asList(l.a(), l.b()),
                    DroneUtil.getCruiseAltitude(getArea())*2.0)));
        }
        overviewPoints.addAll(toAdd);
        overviewPath = new LinkedList<>(optimizeRoutes(overviewPoints)); // since linked lists implement queues

        // And of course we'll eventually have detailed points to fly to
        detailedPath = new LinkedList<>();
    }
}