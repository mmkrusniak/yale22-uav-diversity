package com.mmkrusniak.drone.variable_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Util;
import com.mmkrusniak.drone.Drone;

import java.util.List;

public abstract class VariableHeightDrone extends Drone {

    public VariableHeightDrone(Area area, int id) {
        super(area, id);
    }

    public VariableHeightDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public boolean predict(Detectable d) {
        if(d == null) return false; // we didn't see it; we have no idea
        if(lowestDetected(d) == null) return false;
        return lowestDetected(d).confidence() > Option.get("detection", "threshold", Double.class);
    }

    public Detectable lowestDetected(Detectable p) {
        List<Detectable> superlist = getTeamDetectionHistory();
        List<Detectable> finds = Util.occurrencesOf(p, superlist);
        Detectable best = null;
        for(Detectable f: finds) if(best == null || f.detectedFrom() < best.detectedFrom()) best = f;
        return best;
    }
}
