package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.phys.Route;

import java.util.List;

/**
 * An abstract drone, one that does not make changes to its path during the traversal. It takes a capture every time
 * it reaches a waypoint, but does nothing with the results. It makes predictions based on the single detections it
 * has made.
 */
public abstract class OfflineDrone extends Drone {

    private List<Route> plan;

    /**
     * Initializes the drone with the default energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     */
    public OfflineDrone(Area area, int id) {
        super(area, id);
    }

    /**
     * Initializes the drone with a custom energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     * @param energyBudget Energy budget of the drone.
     */
    public OfflineDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    /**
     * Gives the plan the drone will follow. Offline drones generate a path and stick to it, so offline
     * implementations need only determine how to generate the steps to take initially and OfflineDrone takes care of
     * the rest.
     * @return The route the drone will take.
     */
    public abstract List<Route> generatePlan();

    /**
     * Generates the initial plan when the drone is started.
     */
    @Override
    public void onBegin() {
        if(getArea() == null) return;
        plan = generatePlan();
    }

    /**
     * Moves to a single next waypoint from the pregenerated path. This method is called whenever the drone is ready
     * to move (has reached the last waypoint), so the overall result is that the drone visits its predetermined path.
     */
    @Override
    public void onAwaiting() {
        if(plan == null) return;
        scan();
        if(! plan.isEmpty()) {
            Route next = plan.remove(0);
            if(next == null) return; // it's a feature not a bug
            move(next);
        }
    }

    /** Does nothing, as offline drones do not respond to external changes. */
    @Override
    public void onEnergyDepleted() {
        // Do nothing
    }

    /** Does nothing, as offline drones do not respond to external changes. */
    @Override
    public void onBroadcastReceived(Broadcast broadcast) {
    }

    /** Does nothing, as offline drones do not respond to external changes. */
    @Override
    public void onMissionCancel() {
        // Do nothing
    }

    /** Does nothing, as offline drones do not respond to external changes. */
    @Override
    public void onTick() {
        // Do nothing
    }

    /**
     * Gives whether the drone is finished with its coverage job. In an offline drone, the job is finished when the
     * entire route has been traversed.
     * @return Whether the drone has finished its coverage task.
     */
    @Override
    public boolean isDone() {
        if(getEnergyRemaining() < 0) return true;
        if(plan == null) return false;
        return plan.isEmpty();
    }

    /**
     * Guesses whether the given detectable is a true positive. By default, guesses solely based on the single
     * detection which is passed to it.
     * @param d A detected object.
     * @return Whether that object is guessed to be a true positive.
     */
    @Override
    public boolean predict(Detectable d) {
        for(Detectable e: getTeamDetectionHistory()) {
            if(e.equals(d)) return e.confidence() > Option.get("detection", "threshold", Double.class);
        }
        return false;
    }
}
