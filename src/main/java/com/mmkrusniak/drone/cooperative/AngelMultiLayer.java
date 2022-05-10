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

import javax.management.timer.Timer;
import java.awt.*;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

// move (begin to move to a point)
// route
// scan (take pictures)


/**
 * This is Angel Flores' implentation of a cooperative, density-aware drone coverage procedure. Roughly, it
 * implements a system where the area is split into subregions of high density, which are covered by some number of
 * drones.
 */
public class AngelMultiLayer extends VariableHeightDrone {

    private final static double THOROUGHNESS = 1.0;

    private int state; // 0 = Explore 1 = Observe 2 = Cover
    private List<Route> plowPlan;
    com.mmkrusniak.geom.Polygon subregion;
    int totalNumExploreDrones = 2;
    Timer timeKeeper;
    int totalScans;
    int totalExploreMoves;
    int observeWaitingPeriod = 2;
    int myTotalMoves;
    boolean active = false;
    private Queue<Route> clusterPlan;
    private List<Detectable> objectsDetected;
    List<Drone> droneList;

    private static final int EXPLORE_STATE = 0;
    private static final int OBSERVE_STATE = 1;


    public AngelMultiLayer(Area area, int id) {
        super(area, id);
    }


    public void explorePhase() {
        if (!plowPlan.isEmpty()) {
            Route next = plowPlan.remove(0);
            if (next != null) {
                move(next);
                // will my code be cut off here because the move() calls onAwaiting() which calls this up to here then loop?
                totalExploreMoves++;
                myTotalMoves++;

                // move remaining code to the start of explore phase?
                // if the explore drone has moved twice now activate the drones back at base // for now pair each drones with another at base but for scaling will need to implement something else
                if (myTotalMoves == 2) {
                    activateObserveDrone();
                }

                // sends a broadcast out after each move. This broadcast will update the clusterPlan for the observe drones
                updatePathsBroadcast();

            }
        } else {
            //switch state to Observe ... need to give it a plan to execute
            state = OBSERVE_STATE;

            // keep track of detectable visited in  observe drones, when this drone has finished send broadcast, then they will recieve an updated queue for them to follow
        }
    }

    public void updatePathsBroadcast() {
        Capture exploreScan = scan();

        // find path to each detectable then add it to cluster clusterPlan
        if (!exploreScan.detectables.isEmpty()) {
            for (Detectable d : exploreScan.detectables) {
                if (objectsDetected.contains(d)) continue;
                objectsDetected.add(d);
                if (altitudeNeeded(d) < d.detectedFrom())
                    // should cluster plan be Queue or List? curr List
                    clusterPlan.add(new RouteTo(new Point(d.x(), d.y(), altitudeNeeded(d))));
            }
            // send broadcast with updated clusterPlan
            for (Drone d : droneList) {
                if (d.getID() - 2 == getID()) {
                    transmit(new Broadcast(this, d, "Updated clusterPlan", new ArrayDeque<>(clusterPlan), getTime()));
                    clusterPlan.clear();
                }
            }
        } else {
            // send broadcast with updated plowPlan ... since know detectables in the last route, travel to the closest route to the exploreDrones because do not need to waste time following empty routes
            for (Drone d : droneList) {
                if (d.getID() - 2 == getID()) {
                    transmit(new Broadcast(this, d, "Updated plowPlan", new ArrayList<>(plowPlan), getTime()));
                }
            }
        }
    }


    public void observePhase() {
        // if the route is not empty grab the next route in the plan array and move there
        if (!clusterPlan.isEmpty()) {
            move(clusterPlan.remove()); // removes head of clusterPlan which should be the next waypoint
        } else {
            //continue to move onto the normal plow plan
            if (! plowPlan.isEmpty()) {
                move(plowPlan.remove(0));
            } // otherwise, we're done
        }
    }

    public void activateObserveDrone() {
        // broadcast to activate home drones ... in the future should change away from iterating through list to search for drone
        for (Drone d : droneList) {
            if (d.getID() - 2 == getID()) {
                transmit(new Broadcast(this, d, "Activate Observe Drone", new ArrayList<>(plowPlan), getTime()));
            }
        }
    }

    // cover phase ...
//    public void coverPhase() {
//        if(!clusterPlan.isEmpty()) {
//            move(clusterPlan.remove(0));
//        }
//        else {
//            if(!plowPlan.isEmpty()) {
//                move(plowPlan.remove(0));
//            }
//        }
//    }

    @Override
    public void onBegin() {
        totalExploreMoves = 0;
        totalScans = 0;
        myTotalMoves = 0;
        droneList = getNeighbors();

        objectsDetected = new ArrayList<>();
        clusterPlan = new ArrayDeque<>();

        // if it is drones 3 or 4 they are turned into observed drones off the start. if 0 or 1 explore drones
        if (getID() >= totalNumExploreDrones) {
            // set Observe state
            state = OBSERVE_STATE;

        } else {
            // set Explore state
            state = EXPLORE_STATE;
            active = true;

            // generate plow path strategy
            if (getArea() == null) {
                plowPlan = null;
            } else {
                plowPlan = generateExplorePlan();
            }
        }
    }

    @Override
    public void onTick() {

    }

    /**
     * Moves to a single next waypoint from the pregenerated path. This method is called whenever the drone is ready
     * to move (has reached the last waypoint), so the overall result is that the drone visits its predetermined path.
     */
    @Override
    public void onAwaiting() {
        scan();
        if (active) {
            // might move scans to occur independently in each phase
            totalScans++;

            switch (state) {
                case 0:
                    explorePhase();
                    break;
                case 1:
                    observePhase();
                    break;
                default:
                    // done()
                    break;
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
    @SuppressWarnings("unchecked")
    public void onBroadcastReceived(Broadcast broadcast) {
        if (broadcast.getHeader().equals("Activate Observe Drone")) {
            System.out.println("Drone " + getID() + "Has received a broadcast: " + broadcast.getHeader());
            active = true;
            plowPlan = (List<Route>) broadcast.getPayload();
            state = OBSERVE_STATE;
        }

        if (broadcast.getHeader().equals("Updated clusterPlan")) {
            clusterPlan.addAll((Queue<Route>) broadcast.getPayload());
        }

        if (broadcast.getHeader().equals("Updated plowPlan")) {
            plowPlan = (List<Route>) broadcast.getPayload();
        }

    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void visualize(Graphics g) {

    }

    /**
     * Gives a preplanned route each Explore drone will take
     *
     * @return The route for the drones to take.
     */
    public List<Route> generateExplorePlan() {
        if (getID() > 1) return new ArrayList<>();

        subregion = getPolygon().split(1, 4)[getID()];
        List<Route> result = new ArrayList<>(
                DroneUtil.optimizePlan(
                        DroneUtil.plow(
                                subregion, getArea().getStart(getID()), DroneUtil.getCruiseAltitude(subregion),
                                -subregion.girth().measure())));
        result.add(0, new RouteHead(subregion.base().measure()));
        result.add(new RouteTo(area.getStart(getID())));
        return result;
    }

    private double altitudeNeeded(Detectable d) {
        double k = d.detectedFrom() * Math.abs(d.confidence() - Option.get("detection", "threshold", Double.class)) * (1 / THOROUGHNESS);
        return Math.max(k, 10);
    }

}

