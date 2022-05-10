package com.mmkrusniak.drone.variable_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.Capture;
import com.mmkrusniak.drone.static_height.PlowDrone;
import com.mmkrusniak.phys.Route;
import com.mmkrusniak.phys.RouteTo;

import java.awt.*;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class DirectDrone extends VariableHeightDrone {
    private final static double THOROUGHNESS = 1.0;

    protected Queue<Route> overviewPath;
    protected Queue<Route> detailedPath;
    private List<Detectable> objectsDetected;
    private boolean onOverviewPath;

    public DirectDrone(Area area, int id) {
        super(area, id);
    }

    public DirectDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    private double altitudeNeeded(Detectable d) {
        double k = d.detectedFrom() * Math.abs(d.confidence() - Option.get("detection", "threshold", Double.class)) * (1 / THOROUGHNESS);
        return Math.max(k, 10);
    }

    @Override
    public void onBegin() {
        PlowDrone basis = new PlowDrone(getArea(), getID());
        overviewPath = new LinkedList<>(basis.generatePlan());
        for(Route r: overviewPath) System.out.println(r);
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
            move(next);

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
