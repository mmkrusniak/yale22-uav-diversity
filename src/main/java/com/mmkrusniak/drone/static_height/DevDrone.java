package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.phys.Route;
import com.mmkrusniak.phys.RouteThrough;

import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

public class DevDrone extends OfflineDrone {
    public DevDrone(Area area, int id) {
        super(area, id);
    }

    public DevDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public List<Route> generatePlan() {
        List<Route> result = new ArrayList<>();
        for(int i = 0; i < 10; i++) result.add(new RouteThrough(new Point(10000000, 200)));
        return result;
    }

    @Override
    public void visualize(Graphics g) { }

    @Override
    public boolean predict(Detectable d) {
        d = d.atHeight(0);
        return d.confidence() > 0.5;
    }
}
