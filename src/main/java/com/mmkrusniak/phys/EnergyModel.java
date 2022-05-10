package com.mmkrusniak.phys;

import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Util;

import java.util.function.Function;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

/**
 * Defines energy models to be used to determine how much energy the drone's motion consumes. One is already defined
 * - the Iris drone model, from DiFranco et al. - which can be accessed with the iris() method. Other models can be
 * easily added with empirical measurements for power consumed at various velocities when accelerating, decelerating,
 * and at constant velocity.
 *
 * We do not currently have such empirical data for Mavic line drones, though with the DJI SDK it could be obtained
 * fairly easily.
 */
public class EnergyModel {
    /**
     * A curve which relates the drone's power usage (in watts) to its velocity when the drone is accelerating.
     */
    private Function<Double, Double> accPowerCurve;
    /**
     * A curve which relates the drone's power usage (in watts) to its velocity when the drone is decelerating.
     */
    private Function<Double, Double> decPowerCurve;
    /**
     * A curve which relates the drone's power usage (in watts) to its velocity when its velocity is constant.
     */
    private Function<Double, Double> constPowerCurve;

    /**
     * The drone's maximum acceleration.
     */
    private double maxAcc;
    /**
     * The drone's maximum velocity.
     */
    private double maxVel;

    /**
     * Default constructor for EnergyModel is not used - use a factory defined here instead
     */
    private EnergyModel() {}

    /**
     * Makes an energy model that fits the Iris drone based on empirical measurements from Di Franco et al. We fit
     * fifth-degree curves to the values they found, which is fairly visually accurate.
     * @return An energy model corresponding to the Iris drone.
     */
    public static EnergyModel iris() {
        EnergyModel result = new EnergyModel();
        result.accPowerCurve = fitQuintic(
                new double[]{0.0,5.0,8.5,12.0,14.0,15.0,15.2,15.3},
                new double[]{200.0,210.0,230.0,240.0,250.0,275.0,300.2,325.3});
        result.decPowerCurve = fitQuintic(
                new double[]{15.0,14.0,12.0,9.5,7.5,6.0,3.0,1.0,0.1},
                new double[]{310.0,260.0,220.0,210.0,212.0,215.0,225.0,230.2,235.0,230.0});
        result.constPowerCurve = fitQuintic(
                new double[]{0,2,4,6,8,10,12,14,16},
                new double[]{222.0,220.0,215.0,210.0,205.0,215.0,235.0,280.0,340.0});
        result.maxVel = 15.0;
        result.maxAcc = Option.get("physics", "drag", Double.class) * result.maxVel * result.maxVel;
        return result;
    }

    /**
     * Utility function which fits a fifth-degree curve to data.
     * @param x x values of the data
     * @param y y values of the data
     * @return A fifth-degree curve which fits those data
     */
    private static Function<Double, Double> fitQuintic(double[] x, double[] y) {
        WeightedObservedPoints points = new WeightedObservedPoints();
        for(int i = 0; i < x.length; i++) points.add(x[i],y[i]);
        double[] coef = PolynomialCurveFitter.create(5).fit(points.toList());
        PolynomialFunction result = new PolynomialFunction(coef);
        return result::value;
    }

    /**
     * Gives the cost this energy model
     * @param state The current motion state of the drone
     * @param t The amount of time over which to calculate energy usage
     * @return The amount of energy used, in joules
     */
    public double getCost(MotionState state, double t) {
        double sum = 0;
        double speed = Math.abs(state.vel2D());

        switch (state.getCurrentMotion()) {
            case CONSTANT: sum += t * constPowerCurve.apply(speed); break;
            case ACCELERATING: sum += t * accPowerCurve.apply(speed); break;
            case DECELERATING: sum += t * decPowerCurve.apply(speed); break;
        }

        if(sum < 0) throw new IllegalStateException("Cost of route is under zero - something is wrong with the energy" +
                " curves.");

        if(state.velZ() > 0.1) sum += state.velZ() * 30 / 2.0;
        return sum;
    }

    /**
     * Constrains the motion of the drone to be in accordance with the drone's maximum capabilities, an important
     * factor in calculating energy usage.
     * @param motionState The motion state of the drone to be constrained.
     */
    public void constrain(MotionState motionState) {
        if(motionState.acc2D() > maxAcc) {
            double dir = new Point(motionState.accX(), motionState.accY()).bearing();
            motionState.setAcc2D(dir, maxAcc);
        }
        if(motionState.vel2D() > maxVel + 0.001) {
            double dir = new Point(motionState.velX(), motionState.velY()).bearing();
            motionState.setVel2D(dir, maxVel);
        }
        motionState.setVelZ(Util.constrain(motionState.velZ(), -5, 5));
    }
}
