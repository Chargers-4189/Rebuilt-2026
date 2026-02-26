// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

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
}