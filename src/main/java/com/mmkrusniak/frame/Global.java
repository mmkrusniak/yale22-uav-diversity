package com.mmkrusniak.frame;

import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.drone.cooperative.ConvexOptimDrone;
import com.mmkrusniak.drone.cooperative.MapElitesDrone;
import com.mmkrusniak.drone.cooperative.PSODrone;

import java.io.File;

public class Global {

    public static void defaultOptions(Drone[] drones){

        Option.register("area", "n", Integer.class, "Number of sides", 5, 3, 30, 1);
        Option.register("area", "file", File.class, "Area file");
        Option.register("area", "size", Integer.class, "Width (meters)", 800, 100, 100000, 100);
        Option.register("area", "jaggedness", Double.class, "Jaggedness", 0.2, 0.0, 1.0, 0.1);
        Option.register("dist", "spread", Object.class, "Spread", SpatialDistribution.CATALOG);
        Option.register("objects", "n", Integer.class, "Number of objects", 20, 1, 100, 1);
        Option.allowRefresh("area", "n");
        Option.allowRefresh("dist", "spread");

        Option.register("drone", "n", Integer.class, "Number of drones", 1, 1, 100, 1);
        Option.register("drone", "dest", com.mmkrusniak.geom.Point.class, "Destination",
                new com.mmkrusniak.geom.Point(200, 200));
        Option.register("drone", "home", com.mmkrusniak.geom.Point.class, "Start Location",
                new com.mmkrusniak.geom.Point(10, 10));
        Option.allowRefresh("drone", "n");

        Option.register("detection", "threshold", Double.class, "Confidence threshold", 0.5, 0.0, 1.0, 0.05);
        Option.register("physics", "sim_speed", Integer.class, "Simulation speed", 20, 1, 100, 1);
        Option.register("physics", "drag", Double.class, "Drag coefficient", 0.061, 0.0, 1.0, 0.001);
        Option.register("physics", "kp", Double.class, "Corner rounding", 0.125, 0.0001, 1.0, 0.0);
        Option.register("physics", "ki", Double.class, "PID kI", 0.0, -100.0, 100.0, 0.0000005);
        Option.register("physics", "kd", Double.class, "PID kD", 0.0, -100.0, 100.0, 0.0000005);
        Option.allowRefresh("physics", "drag");

        Option.register("graphics", "render", Boolean.class, "Live rendering", true);
        Option.register("graphics", "live_metrics", Boolean.class, "Live metrics", true);

        Option.register("graphics", "trace", Boolean.class, "Show trace", true);
        Option.depend("graphics", "trace", "graphics", "render", Boolean.TRUE);
        Option.register("graphics", "broadcast_indicators", Boolean.class, "Show transmit range", true);
        Option.depend("graphics", "broadcast_indicators", "graphics", "render", Boolean.TRUE);
        Option.register("graphics", "pdf", Boolean.class, "Visualize probability", true);
        Option.depend("graphics", "pdf", "graphics", "render", Boolean.TRUE);
        Option.register("graphics", "captures", Boolean.class, "Show captures", true);
        Option.depend("graphics", "captures", "graphics", "render", Boolean.TRUE);

        Option.register("drone_team", "0", Drone.class, "Drone #1", drones);

        Option.registerList("dist", "modes", SpatialDistribution.DistMode[].class, "Peaks",
                "Peak", new SpatialDistribution.DistMode(new com.mmkrusniak.geom.Point(0, 0), 100));
        Option.register("dist", "mode", SpatialDistribution.DistMode.class, "Mode",
                new SpatialDistribution.DistMode(new com.mmkrusniak.geom.Point(100, 1000), 100));
        Option.depend("dist", "mode", "dist", "spread", "Gaussian");
        Option.depend("dist", "modes", "dist", "spread", "Multimodal");
    }
}
