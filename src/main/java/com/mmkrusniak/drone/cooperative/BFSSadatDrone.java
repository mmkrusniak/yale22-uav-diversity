package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.variable_height.VariableHeightDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.RouteTo;

import java.awt.*;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

/**
 * This is an implementation of Sadat et al.'s breadth-first-search coverage algorithm, where a single, very
 * high-altitude image is taken of the whole region, then that region is split into smaller rectangular subregions,
 * with subregions containing no objects according to the initial image ignored. This process is done recursively
 * until no drone energy remains.
 *
 * Unfortunately, practically speaking, this procedure is not effective; it takes too much energy to rise to the
 * altitude necessary for the initial image (and this is reflected in the simulator.)
 */
public class BFSSadatDrone extends VariableHeightDrone {

    private final Queue<com.mmkrusniak.geom.Polygon> queue = new LinkedList<>();
    private com.mmkrusniak.geom.Polygon current;
    private static final int INTERESTING_THRESHOLD = 0;
    private static final int BRANCH_FACTOR = 2;
    private boolean done;

    public BFSSadatDrone(Area area, int id) {
        super(area, id);
    }
    public BFSSadatDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public void onBegin() {
        queue.clear();
        done = false;
        queue.add(getPolygon());
        if(getPolygon().getPolygonalHeight() < getPolygon().getCartesianWidth()) setHeading(Math.PI/2);
        else setHeading(0.0);
    }

    @Override
    public void onTick() {

    }

    @Override
    public void onAwaiting() {
        System.out.println("Awaiting");
        List<Detectable> detected = scan().detectables;

        if(current != null && DroneUtil.getScanHeight(current.getCartesianWidth()) > 30) {
            double newWidth = current.getCartesianWidth() / BRANCH_FACTOR;
            double newHeight = current.getCartesianHeight() / BRANCH_FACTOR;
            double oldX = current.leftmost().x();
            double oldY = current.topmost().y();
            for (int i = 0; i < BRANCH_FACTOR; i++) {
                for (int j = 0; j < BRANCH_FACTOR; j++) {
                    com.mmkrusniak.geom.Polygon child = new com.mmkrusniak.geom.Polygon( // probably should have said that polygons are ccw in the docs
                            new com.mmkrusniak.geom.Point(oldX + i*newWidth, oldY + j*newHeight),
                            new com.mmkrusniak.geom.Point(oldX + i*newWidth, oldY + (j+1)*newHeight),
                            new com.mmkrusniak.geom.Point(oldX + (i+1)*newWidth, oldY + (j+1)*newHeight),
                            new com.mmkrusniak.geom.Point(oldX + (i+1)*newWidth, oldY + (j)*newHeight)
                    );
                    int count = 0;
                    for(Detectable d: detected) if(child.encloses(d)) count++;
                    if(count > INTERESTING_THRESHOLD) queue.add(child);
                }
            }
        }
        if(queue.isEmpty()) {
            done = true;
            return;
        }
        current = queue.remove();
        move(new RouteTo(current.getCenter().extend(DroneUtil.getScanAlt(current.getCartesianWidth()))));
    }



    @Override
    public void onMissionCancel() {

    }

    @Override
    public void onEnergyDepleted() {

    }

    @Override
    public void onBroadcastReceived(Broadcast broadcast) {

    }

    @Override
    public boolean isDone() {
        return done;
    }

    @Override
    public void visualize(Graphics g) {
        g.setColor(Color.BLACK);
        for(Polygon p: queue) p.render(g);
        g.setColor(Color.CYAN);
        if(current != null) current.render(g);
    }
}
