package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.variable_height.VariableHeightDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.Capture;
import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;
import com.mmkrusniak.phys.RouteTo;

import java.awt.*;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

/**
 * This class describes a coverage procedure where a number of drones split a path that a hypothetical drone with
 * energy reserves equal to the sum of the group would take. Then, during runtime, objects detected are immediately
 * descended to. (I [Miles] am unsure to what extent that is actually implemented here, but this is the general idea.)
 *
 * The base path is a simple plow motion.
 */
public class EndToEndDirectDrone extends VariableHeightDrone {
    private final static double THOROUGHNESS = 1.0;

    protected Queue<Route> overviewPath;
    protected Queue<Route> detailedPath;
    private List<Detectable> objectsDetected;
    private boolean onOverviewPath;

    public EndToEndDirectDrone(Area area, int id) {
        super(area, id);
    }

    public EndToEndDirectDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    private double altitudeNeeded(Detectable d) {
        double k = d.detectedFrom() * Math.abs(d.confidence() - Option.get("detection", "threshold", Double.class)) * (1 / THOROUGHNESS);
        return Math.max(k, 10);
    }

    @Override
    public void onBegin() {
        overviewPath = new LinkedList<>();
        com.mmkrusniak.geom.Polygon target = getPolygon();

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
            if(totalPath.get(i).getTarget() == null) overviewPath.add(totalPath.get(i));
        }
        for(int i = iStart; i < iStart + totalPath.size()/getTeamSize(); i++) overviewPath.add(totalPath.get(i));
        detailedPath = new LinkedList<>();
        objectsDetected = new ArrayList<>();
        onOverviewPath = true;
    }

    @Override
    public void onAwaiting() {
        Capture c = scan();
        if(onOverviewPath && ! c.detectables.isEmpty() ) {
            for (Detectable d : c.detectables) {
                if (objectsDetected.contains(d)) continue;
                objectsDetected.add(d);
                if (altitudeNeeded(d) < d.detectedFrom())
                    detailedPath.add(new RouteTo(new Point(d.x(), d.y(), altitudeNeeded(d))));
            }
            onOverviewPath = false;
        }
        if(! detailedPath.isEmpty()) move(detailedPath.remove());
        else if(! overviewPath.isEmpty()){
            Route next;
            if(! (overviewPath.peek().getTarget() == null) && ! onOverviewPath) next =
                    new RouteTo(overviewPath.remove().getTarget());
            else next = overviewPath.remove();

            onOverviewPath = true;
            move(next); // this is why Coordinate is a subclass of Point, basically

        } else move(area.getHull().farthest(area.getStart(getID())));
    }

    @Override
    public void onEnergyDepleted() { }

    @Override
    public void onBroadcastReceived(Broadcast broadcast) { }

    @Override
    public boolean isDone() {
        if(overviewPath == null || detailedPath == null) return false; // we haven't even begun
        return overviewPath.isEmpty() && detailedPath.isEmpty();
    }

    @Override
    public void visualize(Graphics g) { }


    @Override
    public void onTick() { }

    @Override
    public void onMissionCancel() { }
}
