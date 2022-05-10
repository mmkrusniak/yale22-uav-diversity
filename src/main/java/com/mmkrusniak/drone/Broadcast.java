package com.mmkrusniak.drone;

/** A broadcast. Is constructed when a drone decides it needs to send a packet.
 *  For now though, this is just a dummy class to represent "I'm a drone who
 *  sent something to someone."
 *  Something to note is that I've set up *some* instance variables to be used for
 *  a later, better implementation, but for now I'll implement something simple that doesn't
 *  use these 'Hello' messages. */
public class Broadcast {
    private final Drone source;
    private final Drone destination;
    private final String header;
    private final Object payload;
    private final int id;
    private static int nextID;
    private final double timestamp;



    public Broadcast(Drone source, Drone destination, String header, Object payload, double timestamp) {
        this.id = nextID++;
        this.source = source;
        this.destination = destination;
        this.header = header;
        this.payload = payload;
        this.timestamp = timestamp;
    }

    /** Return my source. */
    public Drone getSource() {
        return source;
    }
    /** Return my destination. */
    public Drone getDestination() {
        return destination;
    }

    /** Return my ID. */
    public int id() {
        return id;
    }

    public Broadcast rebroadcast(double t) {
        return new Broadcast(source, destination, header, payload, t);
    }

    public Object getPayload() { return payload; }

    public String getHeader() { return header; }
}
