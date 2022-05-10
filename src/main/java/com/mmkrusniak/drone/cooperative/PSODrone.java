package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.drone.variable_height.VariableHeightDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.phys.RouteTo;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class PSODrone extends VariableHeightDrone {


    java.util.List<com.mmkrusniak.geom.Point> path;
    java.util.List<com.mmkrusniak.geom.Point> graphicTrace;
    Point[][] candidates;
    com.mmkrusniak.geom.Point destination;

    final int NUM_SEGS = 10;
    final int NUM_PARTICLES = 50;
    final int LOOPS = 50;
    final double COEF_INERTIA = 0.2;
    final double COEF_LOCAL = 2;
    final double COEF_SOCIAL = 2;


    public PSODrone(Area area, int id) {
        super(area, id);
    }

    private List<Point> getPath() {
        Point[][] particles_v = new Point[NUM_PARTICLES][NUM_SEGS];
        Point[][] particles_x = new Point[NUM_PARTICLES][NUM_SEGS];
        Point[][] particles_b = new Point[NUM_PARTICLES][NUM_SEGS];
        double[] particles_s = new double[NUM_PARTICLES];
        Point[] global_b = new Point[NUM_SEGS];
        double global_s = Double.POSITIVE_INFINITY;

        Line crowLine = new Line(getArea().getStart(getID()), destination);
        List<com.mmkrusniak.geom.Point> base =
                new ArrayList<>((crowLine).toSubpoints(crowLine.length() / (NUM_SEGS-2)));
        base.add(getArea().getDest(getID()));

        // Initialize particles as deviations from a line "as the crow flies"
        for (int i = 0; i < NUM_PARTICLES; i++) {
            for (int j = 0; j < NUM_SEGS; j++) {
                Point basePoint = base.get(j);
                particles_v[i][j] = new Point(Math.random()*10-5, Math.random()*10-5);
                particles_v[i][j] = new Point(0,0);
                particles_x[i][j] = new Point(basePoint.x()+Math.random()*100-50, basePoint.y()+Math.random()*100-50);
                particles_b[i][j] = particles_x[i][j];
            }
        }

        // Initialize global best and particle best

        for (int i = 0; i < NUM_PARTICLES; i++) {
            particles_s[i] = approxPathScore(Arrays.asList(particles_b[i]));
            if(particles_s[i] <= global_s) {
                System.arraycopy(particles_x[i], 0, global_b, 0, NUM_SEGS);
                global_s = particles_s[i];
            }
        }

        // Run PSO
        // For a set number of iterations
        //   (alternative would be "until change in score is low")
        for (int itr = 0; itr < LOOPS; itr++) {
            // For every particle
            for (int i = 0; i < NUM_PARTICLES; i++) {
                // For every path element in that particle
                for (int j = 0; j < NUM_SEGS; j++) {
                    Point x = particles_x[i][j];
                    Point v = particles_v[i][j];
                    Point b = particles_b[i][j];
                    Point g = global_b[j];
                    particles_v[i][j] = new Point (
                            COEF_INERTIA * v.x() +
                                    COEF_LOCAL  * Math.random() * (b.x()-x.x()) +
                                    COEF_SOCIAL * Math.random() * (g.x()-x.x()),
                            COEF_INERTIA * v.y() +
                                    COEF_LOCAL  * Math.random() * (b.y()-x.y()) +
                                    COEF_SOCIAL * Math.random() * (g.y()-x.y())
                    );
                    particles_x[i][j] = new Point (
                            x.x() + particles_v[i][j].x(), x.y() + particles_v[i][j].y()
                    );
                    if(! getArea().contains(particles_x[i][j]) && getArea().getHull().distance(particles_x[i][j]) > 30) {
                        particles_x[i][j] = area.getHull().closest(particles_x[i][j]);
                    }
                }
                double score = approxPathScore(Arrays.asList(particles_x[i]));
                if(score < particles_s[i]) {
                    particles_s[i] = score;
                    System.arraycopy(particles_x[i], 0, particles_b[i], 0, NUM_SEGS);
                }
                if(score < global_s) {
                    global_s = score;
                    System.arraycopy(particles_x[i], 0, global_b, 0, NUM_SEGS);
                }
            }
        }


        int minIndex = 0;
        int min2Index = 1;
        int min3Index = 2; // yes yes i know i should just sort it but it's slightly more efficient this way
        for (int i = 0; i < NUM_PARTICLES; i++) {
            if(particles_s[i] < particles_s[minIndex]) minIndex = i;
            else if(particles_s[i] < particles_s[min2Index]) min2Index = i;
            else if(particles_s[i] < particles_s[min3Index]) min3Index = i;
        }

        int resIndex = getID()==0? minIndex : getID()==1? min2Index : min3Index;

        candidates = particles_b;
        return new ArrayList<>(Arrays.asList(particles_x[resIndex]));
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
        for(Point[] p: candidates) {
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
