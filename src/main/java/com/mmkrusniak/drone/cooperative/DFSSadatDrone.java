package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.variable_height.VariableHeightDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.RouteTo;

import java.awt.*;
import java.util.List;
import java.util.Stack;

/**
 * This is an implementation of the depth-first version of Sadat et al. In this procedure, a single image is taken at
 * very high altitude of the entire region, then the image is recursively split into smaller, rectangular
 * subsections. Subsections with no apparent objects are dismissed and the rest form a tree, which is traversed with
 * a depth-first search.
 *
 * This algorithm behaves poorly: aside from the fact that it is not energy-efficient to reach the altitude required
 * to image the entire area, it also tends to waste energy getting needlessly detailed images of single objects in
 * the tree.
 */
public class DFSSadatDrone extends VariableHeightDrone {

    private final Stack<com.mmkrusniak.geom.Polygon> stack = new Stack<>();
    private com.mmkrusniak.geom.Polygon current;
    private static final int INTERESTING_THRESHOLD = 0;
    private static final int BRANCH_FACTOR = 2;
    private boolean done;

    public DFSSadatDrone(Area area, int id) {
        super(area, id);
    }
    public DFSSadatDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public void onBegin() {
        stack.clear();
        done = false;
        stack.push(getPolygon());

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
                    if(count > INTERESTING_THRESHOLD) stack.push(child);
                }
            }
        }
        if(stack.isEmpty()) {
            done = true;
            return;
        }
        current = stack.pop();
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
        for(Polygon p: stack) p.render(g);
        g.setColor(Color.CYAN);
        if(current != null) current.render(g);
    }
}
