package com.mmkrusniak.drone;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Option;

import javax.swing.text.html.HTMLDocument;
import java.awt.*;
import java.util.*;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.IntConsumer;

public class DroneTeam implements Iterable<Drone> {

    private final Drone[] drones;
    private Thread thread;
    private final List<DroneTeamListener> listeners;

    public DroneTeam(Drone... drones) {
        this.drones = drones;
        listeners = new ArrayList<>();
        for(Drone d: drones) d.joinTeam(this);
    }

    private static long lastUpdateTime = System.nanoTime();

    private static long lastRenderTime = System.nanoTime();
    private static long renderInterval = 1000*1000*1000/30;
    private boolean killThread;

    public synchronized void traverse(final Area area, final Consumer<Drone[]> onTick) {
        long intervalOption = Option.get("physics", "sim_speed", Integer.class);
        long updateInterval = 1000*1000*1000/intervalOption/intervalOption;
        stop();
        for (Drone drone : drones) drone.reset(area);
        thread = new Thread(() -> {
            for(Drone d: drones) d.onBegin();
            while(dronesRunning() && ! killThread) {
                if (lastUpdateTime + updateInterval < System.nanoTime()) {
                    for (Drone d : drones) d.proceed(.2);
                    if(onTick != null) onTick.accept(drones);
                    lastUpdateTime = System.nanoTime();
                }
                if (lastRenderTime + renderInterval < System.nanoTime()) {
                    for(DroneTeamListener l: listeners) l.onTraversalProgressed();
                    lastRenderTime = System.nanoTime();
                }
            }
            if(! killThread) for(DroneTeamListener l: listeners) l.onTraversalFinished();
        }, "path_planning");
        thread.start();
    }

    public void render(Graphics g) {
        for(Drone d: drones) {
            d.render(g);
        }
    }

    @Override
    public Iterator<Drone> iterator() {
        return Arrays.asList(drones).iterator();
    }

    public interface DroneTeamListener {
        void onTraversalFinished();
        void onTraversalProgressed();
    }
    public void addListener(DroneTeamListener listener) {
        listeners.add(listener);
    }

    public boolean dronesRunning() {
        for(Drone d: drones) if(! (d.isDone() || d.getEnergyRemaining() < 0)) return true;
        return false;
    }

    public boolean isTraversing() {
        return (thread != null && thread.isAlive());
    }

    public void stop() {
        if(isTraversing()) {
            killThread = true;
            try { thread.join();
            } catch (InterruptedException e) { e.printStackTrace(); }
            killThread = false;
        }
    }

    public HTMLDocument[] dronesToHTML() {
        HTMLDocument[] result = new HTMLDocument[drones.length];
        for (int i = 0; i < drones.length; i++) {
            result[i] = DroneUtil.stringToHTML(drones[i].toHTML());
        }
        return result;
    }

    public int size() {
        return drones.length;
    }
    public Drone getDrone(int id) {
        return drones[id];
    }
}
