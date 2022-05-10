package com.mmkrusniak.phys;

import com.mmkrusniak.drone.WayPointCheck;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Angle;
import com.mmkrusniak.geom.Point;

import java.util.List;

/**
 * This is a legacy class meant to help implement banked turning. Now, banked turning is more or less implemented
 * as part of the dynamic physics model introduced in 2020, but some of the code here may still be applicable.
 */
public class Turn {

    private  List<Point> wayPoints;
    private  int index;

    public Turn(List<Point> wayPoints, int index){
        this.wayPoints = wayPoints;
        this.index = index;
    }

    public double getAngle(){
        Angle angle = new Angle(
                wayPoints.get((index+1) % wayPoints.size()),
                wayPoints.get(index),
                wayPoints.get((index+wayPoints.size()-1) % wayPoints.size()));
        double angleDouble = angle.measure();
        if(angle.measure()>Math.PI && angle.measure()-Math.PI>0.0001){
            angleDouble = angleDouble - Math.PI;
        }
        return angleDouble;
    }

    public boolean isStopAndTurn(){
        boolean result = false;
        if(!isBankedTurning()){
            result = true;
        }
        return result;
    }

    public double getTurningEnergy(){
        double angle = getAngle();
        double power = Option.turningPower;
        double speed = Option.angleSpeed;
        return power * angle / speed;
    }

    //to ensure that there are a pair of turns left for banked turning
    public boolean isBankedTurning(){
    boolean result = false;
        if(new WayPointCheck(wayPoints,index+1).isATurn()){
            result =true;
        }
        if(new WayPointCheck(wayPoints,index-1).isATurn()){
            result =true;
        }
        return result;
    }

    // to ensure the length between 2 waypoints is enough for banked turning
    public boolean canBeBankedTurning(double speed){
        boolean result = false;
        if(wayPoints.get(index).distance2D(wayPoints.get(index+1)) -2* getBankedTurningRadius(speed)>0){
            result = true;
        }
        return result;
    }

    public double getBankedTurningRadius(double speed){
        double angle = 2 * Math.PI * Option.tiltAngle/360.0;
        double result = Math.pow(speed,2)/(9.8*Math.tan(angle));
        return result;
    }

    public  double getBankedTurningLength(double speed){
        double SemiCircle = Math.PI * getBankedTurningRadius(speed);
        double straight = wayPoints.get(index).distance2D(wayPoints.get(index-1)) -2* getBankedTurningRadius(speed);
        return straight+SemiCircle;
    }
}
