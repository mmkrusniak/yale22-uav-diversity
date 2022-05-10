package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.variable_height.VariableHeightDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Angle;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.phys.RouteTo;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MapElitesDrone extends VariableHeightDrone {


    java.util.List<com.mmkrusniak.geom.Point> path;
    java.util.List<com.mmkrusniak.geom.Point> graphicTrace;
    Point[][] pool;
    com.mmkrusniak.geom.Point destination;

    private int MAX_DRIFT = 100;

    final int NUM_SEGS = 8;
    final int NUM_CLASSES = 15;
    final int LOOPS = 10000;
    public MapElitesDrone(Area area, int id) {
        super(area, id);
    }

    private int classify(Point[] path, Line crowLine) {
        double extremeness = 0;
        for(Point p: path) {
            extremeness += p.distance2D(crowLine);
        }
        extremeness /= path.length;
        int classE = (int) (extremeness/50);
        if(classE > 4) classE = 4;

        int valence = 0;
        for(Angle l: Angle.arrayFromPoints(path)) {
            if(l.measure() > Math.PI) valence += 1;
            else valence -= 1;
        }
        int classV = 0;
        if(valence > NUM_SEGS/2) classV = 1;
        if(valence < -NUM_SEGS/2) classV = 2;

        // three options for valence, five options for extremeness
        int result = classE*3 + classV;
        return result;
    }

    private Point[] mutate(Point[] unit, Point[][] pool) {
        Point[] offspring = new Point[NUM_SEGS];
        int rand = (int) (Math.random() * NUM_SEGS);
        for (int i = 0; i < NUM_SEGS; i++) {
            offspring[i] = new Point(
                    unit[i].x() + (i == rand ? (Math.random()*MAX_DRIFT-MAX_DRIFT/2) : 0),
                    unit[i].y() + (i == rand ? (Math.random()*MAX_DRIFT-MAX_DRIFT/2) : 0)
            );
        }
        return offspring;
    }

    private Point[] pick(Point[][] pool, double[] scores) {
        double scoreSum = 0;
        for (int i = 0; i < scores.length; i++) scoreSum += scores[i];
        System.out.println(scoreSum);
        double random = Math.random() * scoreSum;
        for (int i = 0; i < scores.length; i++) {
            scoreSum -= scores[i];
            if(scoreSum < 0) return pool[i];
        }
        System.err.println("Warning: pick out of range for MAP-ELITES");
        return pool[scores.length-1];
    }

    private List<Point> getPath() {
        pool = new Point[NUM_CLASSES][NUM_SEGS];
        double[] scores = new double[NUM_CLASSES];
        for (int i = 0; i < NUM_CLASSES; i++) scores[i] = Double.POSITIVE_INFINITY;

        Line crowLine = new Line(getArea().getStart(getID()), destination);
        List<com.mmkrusniak.geom.Point> base = new ArrayList<>((crowLine).toSubpoints(crowLine.length() / NUM_SEGS));

        for (int itr = 0; itr < LOOPS; itr++) {
            Point[] unit = pool[(int) (Math.random()*NUM_CLASSES)];
            if(unit[0] == null) {
                unit = new Point[NUM_SEGS];
                for (int j = 0; j < NUM_SEGS; j++) {
                    Point basePoint = base.get(j);
                    unit[j] = new Point(basePoint.x()+Math.random()*300-150, basePoint.y()+Math.random()*300-150);
                }
            }
            unit = mutate(unit, pool);
            double score = approxPathScore(Arrays.asList(unit));
            int classification = classify(unit, crowLine);
            if(score < scores[classification]) {
                scores[classification] = score;
                pool[classification] = unit;
            }
        }

        int minIndex = 0;
        int min2Index = 1;
        int min3Index = 2; // yes yes i know i should just sort it but it's slightly more efficient this way
        for (int i = 0; i < NUM_CLASSES; i++) {
            if(scores[i] < scores[minIndex]) minIndex = i;
            else if(scores[i] < scores[min2Index]) min2Index = i;
            else if(scores[i] < scores[min3Index]) min3Index = i;
        }

        int resIndex = getID()==0? minIndex : getID()==1? min2Index : min3Index;

        return new ArrayList<>(Arrays.asList(pool[resIndex]));
    }

    @Override
    public void onBegin() {
        destination = getArea().getDest(getID());
        path = getPath();
        graphicTrace = new ArrayList<>();
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

        g.setColor(new Color(200, 200, 200, 100));
        for(Point[] p: pool) {
            if(p[0] == null) continue;
            for(Line l: Line.arrayFromPoints(p)) {
                l.render(g);
            }
        }

        g.setColor(Color.RED);
        destination.render(g, com.mmkrusniak.geom.Point.CROSS);

        g.setColor(Color.CYAN);
        for(Point p: graphicTrace) p.render(g);

    }
}
