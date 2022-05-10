package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.static_height.PlowDrone;
import com.mmkrusniak.drone.variable_height.VariableHeightDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.phys.Route;

import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

/**
 * This is an implementation of the "layer" multi-drone strategy: one subteam of drones is responsible for
 * high-altitude imagery, while another subteam directly visits possible objects found by the high-altitude subteam.
 *
 * In this implementation, the first drone forms the entire high-altitude subteam and traverses the area with a
 * simple plow motion, while the other drones form the low-altitude subteam. This works best for areas with high
 * object density.
 */
public class CooperativeLayerDrone extends VariableHeightDrone {

    public CooperativeLayerDrone(Area area, int id) {
        super(area, id);
    }
    public CooperativeLayerDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    private List<Route> model;
    private final List<Detectable> subpoints = new ArrayList<>();
    private final List<Detectable> pending = new ArrayList<>();;
    private boolean done;

    @Override
    public void onBegin() {
        model = new PlowDrone(getArea(), getID()).generatePlan();
        subpoints.clear();
        pending.clear();
        done = false;
    }

    @Override
    public void onTick() {

    }

    @Override
    public void onAwaiting() {

        if(getID() == 0) {
            pending.addAll(scan().detectables);
            if(model.isEmpty()) done = true;
            else {
                pending.addAll(scan().detectables);
                move(model.remove(0));
                List<Detectable> assigned = new ArrayList<>();
                for(Detectable p: pending) {
                    Drone target = null;
                    for(Drone d: getNeighbors()) {
                        if(target == null || d.getLocation().distance2D(p) < target.getLocation().distance2D(p) || d.isDone()) {
                            target = d;
                        }
                    }
                    if(target != null) {
                        transmit(new Broadcast(this, target, "observe", p, getTime()));
                        assigned.add(p);
                    }
                }
                pending.removeAll(assigned);
            }
        }
        else {
            if(subpoints.isEmpty()) {
                done = true;
                if(getNeighbors().size() > 0) {
                    Point p = getNeighbors().get(0).getLocation();
                    move(new Point(p.x(), p.y(), getLocation().z()));
                }
                else move(area.getStart(getID()));
            }
            else {
                done = false;
                scan();
                Detectable next = subpoints.remove(0);
                move(next.extend(altitudeNeeded(next)));
            }
        }
    }

    @Override
    public void onMissionCancel() {

    }

    @Override
    public void onEnergyDepleted() {

    }

    @Override
    public void onBroadcastReceived(Broadcast broadcast) {
        Detectable d = (Detectable) broadcast.getPayload();
        if(! subpoints.contains(d)) subpoints.add(d);
    }

    @Override
    public boolean isDone() {
        return done;
    }

    @Override
    public void visualize(Graphics g) {

    }

    private double altitudeNeeded(Detectable d) {
        double k = d.detectedFrom() * Math.abs(d.confidence() - Option.get("detection", "threshold", Double.class));
        return Math.max(k, 10);
    }
}
