package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.TreeMap;
import java.util.Map;

/**
 * Manages shooter calibration data and provides interpolated settings for aiming.
 */
public class ShooterTable {

    /**
     * A simple data structure to hold the power and hood setting for a shot.
     */
    public static class ShotParams {
        public final double power;
        public final double hood;

        public ShotParams(double power, double hood) {
            this.power = power;
            this.hood = hood;
        }
    }

    // This is the lookup table where you will store your final calibration data.
    // The key is the distance (in inches), and the value contains the best power/hood for that distance.
    private static final TreeMap<Double, ShotParams> lookupTable = new TreeMap<>();

    // The static block is where you will input your data.
    static {
        // ===================================================================================
        // ---!!!---           POPULATE THIS TABLE WITH YOUR CALIBRATION DATA          ---!!!--- 
        // ===================================================================================
        // For each distance you tested, find the best-performing shot (or average a few good ones)
        // and add it to the table here.

        /*
         *  EXAMPLE DATA - REPLACE WITH YOUR OWN
         */
        lookupTable.put(48.0, new ShotParams(0.600, 0.400));
        lookupTable.put(72.0, new ShotParams(0.750, 0.550));
        lookupTable.put(96.0, new ShotParams(0.900, 0.700));
        // lookupTable.put(120.0, new ShotParams(0.950, 0.750)); // Add more points for more accuracy

    }

    /**
     * Calculates the ideal shooter power and hood position for a given distance using linear interpolation.
     * @param distanceToTarget The distance to the target in inches, measured by the Limelight.
     * @return A ShotParams object containing the calculated power and hood position.
     */
    public static ShotParams getInterpolatedShot(double distanceToTarget) {


        // Find the known data points immediately below and above the current distance
        Map.Entry<Double, ShotParams> lowerEntry = lookupTable.floorEntry(distanceToTarget);
        Map.Entry<Double, ShotParams> upperEntry = lookupTable.ceilingEntry(distanceToTarget);

        // Handle edge cases where the distance is outside the calibrated range
        if (lowerEntry == null) {
            return upperEntry.getValue(); // Target is closer than our closest data point, use the closest setting
        }
        if (upperEntry == null) {
            return lowerEntry.getValue(); // Target is further than our furthest data point, use the furthest setting
        }

        // If an exact match is found, or if the two points are the same
        if (lowerEntry.getKey().equals(upperEntry.getKey())) {
            return lowerEntry.getValue();
        }
        
        // --- Perform Linear Interpolation ---
        double lowerDist = lowerEntry.getKey();
        double upperDist = upperEntry.getKey();
        ShotParams lowerShot = lowerEntry.getValue();
        ShotParams upperShot = upperEntry.getValue();

        // Calculate how far the target distance is between the two known data points (0.0 to 1.0)
        double range = upperDist - lowerDist;
        double proportion = (distanceToTarget - lowerDist) / range;

        // Apply that proportion to the power and hood values
        double interpolatedPower = lowerShot.power + proportion * (upperShot.power - lowerShot.power);
        double interpolatedHood = lowerShot.hood + proportion * (upperShot.hood - lowerShot.hood);

        return new ShotParams(interpolatedPower, interpolatedHood);
    }
}
