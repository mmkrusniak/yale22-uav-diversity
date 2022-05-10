package com.mmkrusniak.geom;

public class Coordinate extends Point {

    public double getLatitude() { return x(); }
    public double getLongitude() { return y(); }
    public double getAltitude() { return z(); }

    private final double heading;
    private double curveSize;

    public Coordinate(double lat, double lon, double alt, double head) {
        super(lat, lon, alt);
        this.heading = head;
    }

    public Coordinate(double standardLongitude, double standardLatitude, double xDistance, double yDistance,
                      double altitude, double heading, double curveSize){
        super(
                standardLongitude+doLngDegress(xDistance, standardLatitude),
                standardLatitude + doLatDegress(yDistance),
                altitude);
        this.heading = heading;
        this.curveSize = curveSize;

    }

    private static Double doLngDegress(double distance,Double latitude) {
        double lngDegree = 2 * Math.asin(Math.sin((double)distance/12742000)/Math.cos(latitude));
        // 转换弧度
        lngDegree = lngDegree * (180/Math.PI);
        System.out.println(lngDegree);
        return lngDegree;
    }

    private static Double doLatDegress(double distance) {
        double latDegrees = (double)distance/6371000;
        // 转换弧度
        latDegrees = latDegrees * (180/Math.PI);
        System.out.println(latDegrees);
        return latDegrees;
    }

    public double getHeading(){
        return heading;
    }
    public double getCurveSize(){
        return curveSize;
    }

}
