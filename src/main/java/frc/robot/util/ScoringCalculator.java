// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ScoringCalculator {
    

    /**
     * Calculates the hood angle to score, given a distance in meters.
     * @param distance distance to the hub (meters)
     * @return hood rotation (encoder difference from the bottom)
     */
    public static double calculateHoodAngle(double distance) {
        return .05 * distance - .05;
    }

    /**
     * Calculates the the shooter power to score, given a distance in meters.
     * @param distance distance to the hub (meters)
     * @return shooting power
     */
    public static double calculateShootingPower(double distance) {
        return 5.286 * distance + 34;
    }

    // public static double[] distances = {47.5, 71.5, 95.5, 119.5, 143.5, 167.5};
    // public static double[] velocities = {43, 48, 51, 55, 56, 59};
    // public static double[] hoodAngles = {0, 0, 0.065, 0.065, 0.095, 0.135};

    // /**
    //  * Calculates the hood angle to score, given a distance in meters.
    //  * @param distance distance to the hub (meters)
    //  * @return hood rotation (encoder difference from the bottom)
    //  */
    // public static double calculateHoodAngle(double distance) {
    //     return superInterpolate(distance, hoodAngles);
    // }

    // /**
    //  * Calculates the the shooter power to score, given a distance in meters.
    //  * @param distance distance to the hub (meters)
    //  * @return shooting power
    //  */
    // public static double calculateShootingPower(double distance) {
    //     return superInterpolate(distance, velocities);
    // }

    // private static double superInterpolate(double distance, double[] values) {
    //     double dist_inches = Units.metersToInches(distance) - 13.5;
    //     if (dist_inches < distances[0]) {
    //         return values[0];
    //     } else if (dist_inches < distances[1]) {
    //         return indexInterpolate(dist_inches, 0, values);
    //     } else if (dist_inches < distances[2]) {
    //         return indexInterpolate(dist_inches, 1, values);
    //     } else if (dist_inches < distances[3]) {
    //         return indexInterpolate(dist_inches, 2, values);
    //     } else if (dist_inches < distances[4]) {
    //         return indexInterpolate(dist_inches, 3, values);
    //     } else if (dist_inches < distances[5]) {
    //         return indexInterpolate(dist_inches, 4, values);
    //     } else {
    //         return values[5];
    //     }
    // }

    // private static double indexInterpolate(double dist_inches, int i, double values[]) {
    //     return interpolate(values[i], values[i+1], (dist_inches - distances[i]) / 24);
    // }

    // private static double interpolate(double value1, double value2, double proportion) {
    //     return value1 * proportion + value2 * (1 - proportion);
    // }

}