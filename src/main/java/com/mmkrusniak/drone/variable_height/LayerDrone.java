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

public class LayerDrone extends VariableHeightDrone {
    private final static double THOROUGHNESS = 1.0;

    private Queue<Route> overviewPath;
    private Queue<Route> detailedPath;
    private List<Detectable> done;

    public LayerDrone(Area area, int id) {
        super(area, id);
    }

    public LayerDrone(Area area, int id, double energyBudget) {
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
        detailedPath = new LinkedList<>();
        done = new ArrayList<>();
        move(overviewPath.remove());
    }

    @Override
    public void onAwaiting() {
        Capture c = scan();
        if (!overviewPath.isEmpty()) {
            if (!c.detectables.isEmpty()) {
                for (Detectable d : c.detectables) {
                    if (done.contains(d)) continue;
                    done.add(d);
                    if (altitudeNeeded(d) < d.detectedFrom()) {
                        detailedPath.add(new RouteTo(new Point(d.x(), d.y(), altitudeNeeded(d))));
                    }
                }
            }
            move(overviewPath.remove());
        } else if(! detailedPath.isEmpty()) {
            move(detailedPath.remove());
        } else move(area.getHull().farthest(area.getStart(getID())));
    }


    @Override
    public void onEnergyDepleted() { }

    @Override
    public void onBroadcastReceived(Broadcast broadcast) { }

    @Override
    public void visualize(Graphics g) { }

    @Override
    public boolean isDone() {
        return overviewPath.isEmpty() && detailedPath.isEmpty();
    }

    @Override
    public void onTick() { }

    @Override
    public void onMissionCancel() { }
}
