package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.phys.Route;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class NullDrone extends OfflineDrone {
    public NullDrone(Area area, int id) {
        super(area, id);
    }

    public NullDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public List<Route> generatePlan() {
        return new ArrayList<>();
    }

    @Override
    public void visualize(Graphics g) {

    }
}
