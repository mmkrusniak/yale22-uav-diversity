package com.mmkrusniak.frame;

import com.mmkrusniak.drone.Capture;
import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.drone.DroneTeam;
import com.mmkrusniak.drone.cooperative.ConvexOptimDrone;
import com.mmkrusniak.drone.cooperative.MapElitesDrone;
import com.mmkrusniak.drone.cooperative.PSODrone;
import com.mmkrusniak.geom.Point;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

public class TextMain {

    public static void main(String[] args) {
        multiUAVScenario(true);
    }

    public static void singleUAVScenario() {
        Drone[] drones = new Drone[] {};
        Global.defaultOptions(drones);
        int nPeaks = 8;

        for (int i = 0; i < 100; i++) {
            Area area = new Area(6);
            Point[] peaks = new Point[nPeaks];
            double[] sds = new double[nPeaks];
            for (int j = 0; j < nPeaks; j++) {
                peaks[j] = area.getHull().random();
                sds[j] = Math.random() * 50 + 50;
            }
            SpatialDistribution d = SpatialDistribution.multimodal(area.getHull(), peaks, sds);
            area.setDistribution(d);

            Drone[] drones1 = new Drone[]{new ConvexOptimDrone(area, 0)};
            DroneTeam team1 = new DroneTeam(drones1);
            Drone[] drones2 = new Drone[]{new PSODrone(area, 0)};
            DroneTeam team2 = new DroneTeam(drones2);
            Drone[] drones3 = new Drone[]{new MapElitesDrone(area, 0)};
            DroneTeam team3 = new DroneTeam(drones3);

            team1.traverse(area, null);
            team2.traverse(area, null);
            team3.traverse(area, null);
            while(team1.isTraversing() || team2.isTraversing() || team3.isTraversing()) ;
            System.out.print(drones1[0].getPathScore() + "\t");
            System.out.print(drones2[0].getPathScore() + "\t");
            System.out.println(drones3[0].getPathScore());
        }
    }

    public static void multiUAVScenario(boolean scenario3) {
        Drone[] drones = new Drone[] {};
        Global.defaultOptions(drones);
        int nPeaks = 10;

        for (int i = 0; i < 100; i++) {
            Area area = new Area(6);
            Point[] peaks = new Point[nPeaks];
            double[] sds = new double[nPeaks];
            Point[] peaksActive = new Point[nPeaks/2];
            double[] sdsActive = new double[nPeaks/2];
            for (int j = 0; j < nPeaks; j++) {
                peaks[j] = area.getHull().random();
                sds[j] = Math.random() * 40 + 10;
                if(j%2==1) {
                    peaksActive[j/2] = peaks[j];
                    sdsActive[j/2] = sds[j];
                }
            }
            SpatialDistribution d = SpatialDistribution.multimodal(area.getHull(), peaks, sds);
            SpatialDistribution r = SpatialDistribution.multimodal(area.getHull(), peaksActive, sdsActive);
            area.setDistribution(d);

            Drone[] drones1 = new Drone[]{new ConvexOptimDrone(area, 0), new ConvexOptimDrone(area, 1),
                    new ConvexOptimDrone(area, 2)};
            DroneTeam team1 = new DroneTeam(drones1);
            Drone[] drones2 = new Drone[]{new PSODrone(area, 0), new PSODrone(area, 1), new PSODrone(area, 2)};
            DroneTeam team2 = new DroneTeam(drones2);
            Drone[] drones3 = new Drone[]{new MapElitesDrone(area, 0), new MapElitesDrone(area, 1),
                    new MapElitesDrone(area, 2)};
            DroneTeam team3 = new DroneTeam(drones3);

            Consumer<Drone[]> consumer = dActive -> {
                for(Drone d1 : dActive) {
                    double prob = r.getDensity(d1.getLocation());
                    if(Math.random() < prob) {
//                        System.out.println("Drone failure: " + d1 + d1.getID());
                        d1.forceHalt();
                    }
                }
            };

            team1.traverse(area, consumer);
            team2.traverse(area, consumer);
            team3.traverse(area, consumer);
            while(team1.isTraversing() || team2.isTraversing() || team3.isTraversing()) ;
            System.out.print(getScenario2Score(drones1) + "\t");
            System.out.print(getScenario2Score(drones2) + "\t");
            System.out.println(getScenario2Score(drones3));
            if(scenario3) {
                System.out.print(getScenario3Score(drones1) + "\t");
                System.out.print(getScenario3Score(drones2) + "\t");
                System.out.println(getScenario3Score(drones3));
            }
        }
    }

    public static String getScenario2Score(Drone[] drones) {
        double result = 800000000;
        for(Drone d: drones) {
            if(d.getEnergyRemaining() <= 0) {
                continue;
            }
            double ds = d.getPathScore();
            if(ds < result) result = ds;
        }
        if(result == 800000000) return "FAILURE";
        return result + "";
    }


    public static String getScenario3Score(Drone[] drones) {
        Set<Detectable> detected = new HashSet<>();
        for(Drone d: drones) for(Capture c: d.getCaptureHistory()) for(Detectable e: c.detectables) {
            if(detected.contains(e)) System.out.println("Already have detectable");
            else {
                System.out.println("Got " + e);
                detected.add(e);
            }
        }
        return detected.size() + "";
    }
}
