package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.Capture;
import com.mmkrusniak.drone.variable_height.VariableHeightDrone;
import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.phys.RouteTo;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class ConvexOptimDrone extends VariableHeightDrone {

    List<Point> path;
    List<Point> graphicTrace;
    Point destination;

    final int NUM_SEGS = 10;
    final int LOOPS = 100;
    final int DELTA = 5;
    final double THRESH = 100;

    public ConvexOptimDrone(Area area, int id) {
        super(area, id);
    }

    @Override
    public void onBegin() {
        path = new LinkedList<>();
        graphicTrace = new ArrayList<>();
        destination = getArea().getDest(getID());
        Line crowLine = new Line(getArea().getStart(getID()), destination);
        List<Point> newPath = new ArrayList<>((crowLine).toSubpoints(crowLine.length() / NUM_SEGS));
        newPath.add(destination); // otherwise omitted from the line
        path.addAll(newPath);

        double oldScore;
        double score = approxPathScore(newPath);

        for (int itr = 0; itr < LOOPS; itr++) {
            for (int i = 0; i < newPath.size(); i++) {
                do {
                    oldScore = score;
                    Point newPoint = new Point(path.get(i).x()+ DELTA, path.get(i).y());
                    path.set(i, newPoint);
                    score = approxPathScore(path);
                } while(score +THRESH < oldScore && path.get(i).x() < area.getHull().rightmost().x());
                do {
                    oldScore = score;
                    Point newPoint = new Point(path.get(i).x()- DELTA, path.get(i).y());
                    path.set(i, newPoint);
                    score = approxPathScore(path);
                } while(score +THRESH < oldScore && path.get(i).x() > area.getHull().leftmost().x());
                do {
                    oldScore = score;
                    Point newPoint = new Point(path.get(i).x(), path.get(i).y()+ DELTA);
                    path.set(i, newPoint);
                    score = approxPathScore(path);
                } while(score +THRESH < oldScore && path.get(i).y() < area.getHull().bottommost().y());
                do {
                    oldScore = score;
                    Point newPoint = new Point(path.get(i).x(), path.get(i).y()- DELTA);
                    path.set(i, newPoint);
                    score = approxPathScore(path);
                } while(score +THRESH < oldScore && path.get(i).y() > area.getHull().topmost().y());
            }
        }
        path.remove(path.size()-1);
        graphicTrace.addAll(path);
    }

    @Override
    public void onTick() {

    }

    @Override
    public void onAwaiting() {
        if(path.isEmpty()) move(new RouteTo(destination));
        else {
            scan();
            move(new RouteTo(path.remove(0)));
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

    }

    @Override
    public boolean isDone() {
        return getLocation().distance2D(destination) < 1;
    }

    @Override
    public void visualize(Graphics g) {
        if(destination == null) return;
        g.setColor(Color.RED);
        destination.render(g, Point.CROSS);

        g.setColor(Color.CYAN);
        for(Point p: graphicTrace) p.render(g);
    }
}
