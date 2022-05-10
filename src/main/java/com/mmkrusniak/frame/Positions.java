package com.mmkrusniak.frame;

import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Polygon;

import java.io.*;
import java.util.Random;


/**
 * A collection of distributions for locations of objects. It's essentially a utility class for Area; it's used to
 * determine where detectables are placed.
 */
public class Positions {

    /**
     * Distributes points pseudorandomly within a polygon. In this distribution a point is as likely to be in one
     * place as it is to be in any other; any clustering is completely coincidental.
     * @param n The number of objects to distribute.
     * @param poly A polygon in which to distribute points.
     * @return An array of locations distributed pseudorandomly.
     */
    public static Point[] pseudorandom(int n, Polygon poly) {
        Point[] result = new Point[n];
        for (int i = 0; i < n; i++) {
            Point p = new Point(
                    Math.random() * poly.getCartesianBounds().width + poly.leftmost().x(),
                    Math.random() * poly.getCartesianBounds().height + poly.topmost().y()
            );
            if (!poly.encloses(p)) i--;
            else result[i] = p;
        }
        return result;
    }

    /**
     * Distributes points approximately according to a Gaussian distribution, where they are more likely to be closer
     * to the center of the polygon. Since a real Gaussian distribution is infinite, this one is truncated; points
     * generated outside the polygon are regenerated until they are all within the polygon.
     * @param n The number of points to generate.
     * @param poly A polygon in which to generate those points.
     * @return A list of points selected by a Gaussian distribution from the center of the polygon.
     */
    //    Real Gaussian
    public static Point[] gaussian(int n, Polygon poly) {
        Point[] result = new Point[n];
        Random rx = new Random();
        Random ry = new Random();
        Point center = poly.getCenter();
        for (int i = 0; i < n; i++) {
            Point p = new Point(
                    poly.getCartesianWidth()*rx.nextGaussian()/4 + center.x(),
                    poly.getCartesianHeight()*rx.nextGaussian()/4 + center.y()
            );
            if (!poly.encloses(p)) i--;
            else result[i] = p;
        }
        return result;
    }

    //TODO This seems hyperspecific to one of Yang's files; we should generalize it.
    /**
     * Distributes points according to predetermined values in a file. A work in progress.
     * @param index ???
     * @return Points distributed according to a file.
     */
    public static double[][] getFromTXT(int index)  {
        int i = 0;
        int k = 0;
        int[] num ={12,2,7,4,322,930,298,84,4,31};
        double[][] results = new double[num[index-1]][2];
        try {
            String file = "/home/yangzhang/LBAI_mega_test/log.txt";
            BufferedReader reader = new BufferedReader(new FileReader(file));
            String line = reader.readLine();
            String[] name = new String[10];
            while(line != null) {
                boolean status = line.contains("jpg");
                if(status){
                    name[i] = line;
                    i++;
                    k = 0;
                }
                else {
                    if(i == index){
                        String[] strs = line.split("=");
                        double x = Double.parseDouble(strs[1].split(" ")[0]);
                        double y = Double.parseDouble(strs[2]);
                        results[k][0] = x;
                        results[k][1] = y;
                        k++;
                    }
                }
                line = reader.readLine();

            }
            reader.close();
            return results;
        } catch (IOException e) {
            e.printStackTrace();
        }
        return results;
    }

    /**
     * Distributes points heuristically in clusters. Typically, each cluster contains between two and nine objects
     * within a square less than one eighth the width of the polygon.
     * @param n The number of points to generate.
     * @param poly The polygon in which to generate those points.
     * @return An array of points which occur in clusters within the polygon.
     */
    public static Point[] clustered(int n, Polygon poly)  {
        Point[] result = new Point[n];

        int i = 0;
        while(i < n) {
            int clusterSize = Math.min((int) (Math.random()*8 + 2), n - i);
            double clusterWidth = poly.getPolygonalWidth() / 8 * Math.random() + 5;
            Point center = pseudorandom(1, poly)[0];

            for(int j = 0; j < clusterSize; j++){
                Point p = pseudorandom(1, center.toSquare(clusterWidth))[0];

                if(! poly.encloses(p)) j--;
                else {
                    result[i] = p;
                    i++;
                }
            }
        }
        return result;
    }
}
