package com.mmkrusniak.drone;

import com.mmkrusniak.geom.Angle;
import com.mmkrusniak.geom.Point;

import java.util.List;

public class WayPointCheck {
    private final List<Point> wayPoints;
    private final int index;

    public WayPointCheck(List<Point> wayPoints,int index){
        this.wayPoints = wayPoints;
        this.index = index;
    }

    public double getAngle() {
        Angle angle = new Angle(wayPoints.get(index+1),wayPoints.get(index),wayPoints.get(index-1));
        double angleDouble = angle.measure();
        if(angle.measure()>Math.PI && angle.measure()-Math.PI>0.0001){
            angleDouble = angleDouble - Math.PI;
        }
        return angleDouble;
    }

    public boolean isATurn() {
        return index != 0
                || index != wayPoints.size()-1
                || ! isCloseTo(getAngle(), Math.PI)
                || ! isCloseTo(getAngle(), 0);
    }


    public boolean isCloseTo(double a, double b) {
        return Math.abs(b-a) < 0.001;
    }



}
