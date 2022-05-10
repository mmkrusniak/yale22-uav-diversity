package com.mmkrusniak.geom;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

public class csvCreate {
    private List<Point> predecideWayPoints;
    List<Coordinate> coordinates = new ArrayList<>();
    private double standardLatitude;//y
    private double standardLongitude;//x
    private double standaX;
    private double standaY;
    public csvCreate(List<Point> predecideWayPoints){
        this.predecideWayPoints = predecideWayPoints;
        standardLatitude = 38.947811;
        standardLongitude = -92.290529;
        standaX = predecideWayPoints.get(0).y();
        standaY  = predecideWayPoints.get(0).x();
        for(Point data : predecideWayPoints){
            coordinates.add(new Coordinate(standardLongitude,standardLatitude,data.x()-standaX, data.y()-standaY,30,0,0));
        }
    }

    public void writeCSV( String finalPath) {
        FileOutputStream out = null;
        OutputStreamWriter osw = null;
        BufferedWriter bw = null;
        try {
            File finalCSVFile = new File(finalPath);
            out = new FileOutputStream(finalCSVFile);
            osw = new OutputStreamWriter(out, "UTF-8");
            // 手动加上BOM标识
            osw.write(new String(new byte[]{(byte) 0xEF, (byte) 0xBB, (byte) 0xBF}));
            bw = new BufferedWriter(osw);
            /**
             * 往CSV中写新数据
             */
            String title = "";
            title = "latitude,longitude,altitude(m),heading(deg),curvesize(m),rotationdir,gimbalmode," +
                    "gimbalpitchangle,actiontype1,actionparam1,actiontype2,actionparam2,actiontype3," +
                    "actionparam3,actiontype4,actionparam4,actiontype5,actionparam5,actiontype6,actionparam6," +
                    "actiontype7,actionparam7,actiontype8,actionparam8,actiontype9,actionparam9,actiontype10," +
                    "actionparam10,actiontype11,actionparam11,actiontype12,actionparam12,actiontype13,actionparam13," +
                    "actiontype14,actionparam14,actiontype15,actionparam15,altitudemode,speed(m/s),poi_latitude," +
                    "poi_longitude,poi_altitude(m),poi_altitudemode,photo_timeinterval,photo_distinterval";
            bw.append(title).append("\r");

            if (coordinates != null && !coordinates.isEmpty()) {
                for (Coordinate data : coordinates) {
                    bw.append(String.valueOf(data.getLatitude())).append(",");
                    bw.append(String.valueOf(data.getLongitude())).append(",");
                    bw.append(data.getAltitude() + ",");
                    bw.append(data.getHeading() + ",");
                    bw.append(data.getCurveSize() + ",");
                    for(int i=0;i<41;i++){
                        bw.append(0 + ",");
                    }
                    bw.append("\r");
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {

            if (bw != null) {
                try {
                    bw.close();
                    bw = null;
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            if (osw != null) {
                try {
                    osw.close();
                    osw = null;
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            if (out != null) {
                try {
                    out.close();
                    out = null;
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

        }
        System.out.println(finalPath + "数据导出成功");
    }
}

