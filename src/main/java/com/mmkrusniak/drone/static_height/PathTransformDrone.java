package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Util;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
/**
 * An offline drone which decomposes an area into a grid, then traverses the area cell by cell according to each
 * cell's distance from an end point and the edges of the area. Due to its high number of turns, it isn't an effective
 * algorithm for UAVs, though it can handle highly complex areas.
 */
public class PathTransformDrone extends OfflineDrone {

    private com.mmkrusniak.geom.Point start, end;

    /** The grid which represents the area. */
    protected Integer[][] grid;
    protected Integer[][] gridRender;

    /**
     * Initializes the drone with the default energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     */
    public PathTransformDrone(Area area, int id) {
        super(area, id);
    }

    /**
     * Initializes the drone with a custom energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     * @param energyBudget Energy budget of the drone.
     */
    public PathTransformDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    private void initGrid() {
        double areaWidth = getPolygon().getCartesianBounds().width;
        double areaHeight = getPolygon().getCartesianBounds().height;
        double imageWidth = DroneUtil.getScanWidth(DroneUtil.getCruiseAltitude(getArea()));
        double imageHeight = DroneUtil.getScanHeight(DroneUtil.getCruiseAltitude(getArea()));

        grid = new Integer[(int) (areaWidth /imageWidth+1)][(int) (areaHeight /imageHeight+1)];

        end = toGrid(area.getHull().farthest(area.getStart(getID())));
        start = toGrid(area.getStart(getID()));
        for (int x = 0; x < grid.length; x++) {
            for (int y = 0; y < grid[x].length; y++) {
                com.mmkrusniak.geom.Point p = toReal(new com.mmkrusniak.geom.Point(x, y));
                if(! area.getHull().encloses(p)) grid[x][y] = null;
                else {
                    grid[x][y] = - (int) p.distance2D(area.getStart(getID()));
                    grid[x][y] -= 2*(int) p.distance2D(area.getHull().closest(p));
                }
            }
        }
        gridRender = grid.clone();
        for (int i = 0; i < gridRender.length; i++) {
            gridRender[i] = gridRender[i].clone();
        }
    }

    /**
     * Gives a preplanned route this drone will take (as it is an offline drone).
     * @return The route for the drone to take.
     */
    @Override
    public List<Route> generatePlan() {
        initGrid();
        List<com.mmkrusniak.geom.Point> result = new ArrayList<>();
        result.add(toReal(start));

        com.mmkrusniak.geom.Point current = start;
        while(! current.equals(end)) {

            com.mmkrusniak.geom.Point bestPoint = null;
            int bestValue = Integer.MIN_VALUE;

            for(int i = 0; (bestPoint == null || bestPoint.equals(end)) && i < grid.length; i++) {
                List<com.mmkrusniak.geom.Point> candidates = Util.adjacent(grid, current, i);

                for (com.mmkrusniak.geom.Point p : candidates) {
                    if(outOfBounds(p)) continue;
                    if (grid[p.ix()][p.iy()] == null) continue;
                    if (bestPoint == null
                            || grid[p.ix()][p.iy()] > bestValue) {
                        bestPoint = p;
                        bestValue = grid[p.ix()][p.iy()];
                    }
                }
            }

            current = bestPoint;
            if(current == null) break; // how does this happen?
            grid[bestPoint.ix()][bestPoint.iy()] = null;
            result.add(toReal(bestPoint));
        }

        List<Route> routes = DroneUtil.optimizePlan(result);
        routes.add(0, new RouteHead(Math.PI/2));
        return routes;
    }

    private boolean outOfBounds(com.mmkrusniak.geom.Point p) {
        if(p.ix() < 0 || p.iy() < 0 || p.ix() >= grid.length || p.iy() >= grid[0].length) return true;
        p = toReal(p);
        return ! getPolygon().encloses(new com.mmkrusniak.geom.Point(p.x(), p.y()));
    }

    private com.mmkrusniak.geom.Point toGrid(com.mmkrusniak.geom.Point p) {
        double imageWidth = DroneUtil.getScanWidth(DroneUtil.getCruiseAltitude(getArea()));
        double imageHeight = DroneUtil.getScanHeight(DroneUtil.getCruiseAltitude(getArea()));
        return new com.mmkrusniak.geom.Point(p.x() / imageWidth, p.y() / imageHeight);
    }
    private com.mmkrusniak.geom.Point toReal(com.mmkrusniak.geom.Point p) {
        double imageWidth = DroneUtil.getScanWidth(DroneUtil.getCruiseAltitude(getArea()));
        double imageHeight = DroneUtil.getScanHeight(DroneUtil.getCruiseAltitude(getArea()));
        return new Point(p.x() * imageWidth + imageWidth/2.0, p.y() * imageHeight + imageHeight/2.0, DroneUtil.getCruiseAltitude(getArea()));
    }

    /**
     * Paints a visualization of the drone on the supplied graphics instance. Good to modify for debugging; otherwise
     * does nothing to reduce visual clutter.
     * @param g A Java graphics canvas on which to paint.
     */
    @Override
    public void visualize(Graphics g) {
//        for (int x = 0; x < gridRender.length; x++) {
//            for (int y = 0; y < gridRender[x].length; y++) {
//                Point p = toReal(new Point(x, y));
//                g.drawString("" + gridRender[x][y], p.ix(), p.iy());
//            }
//        }
    }
}
