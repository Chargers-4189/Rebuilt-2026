// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.util.NetworkTables.PassingCalculatorTable;
import frc.robot.util.NetworkTables.ShootingCalculatorTable;

/** Add your docs here. */
public class ScoringCalculator {
    

    /**
     * Calculates the hood angle to score, given a distance in meters.
     * @param distance distance to the hub (meters)
     * @return hood rotation (encoder difference from the bottom)
     */
    public static double calculateShootingAngle(double distance) {
        return ShootingCalculatorTable.kAngleSlope.get() * distance + ShootingCalculatorTable.kAngleIntercept.get();
    }

    /**
     * Calculates the the shooter power to score, given a distance in meters.
     * @param distance distance to the hub (meters)
     * @return shooting power
     */
    public static double calculateShootingPower(double distance) {
        return ShootingCalculatorTable.kVelocitySquared.get() * distance * distance + ShootingCalculatorTable.kVelocitySlope.get() * distance + ShootingCalculatorTable.kVelocityIntercept.get();
    }

    public static double calculatePassingPower(double distance) {
        return ShootingCalculatorTable.kVelocitySquared.get() * distance * distance + PassingCalculatorTable.kVelocitySlope.get() * distance + PassingCalculatorTable.kVelocityIntercept.get();
    }

    public static double calculatePassingAngle(double distance) {
        return PassingCalculatorTable.kAngleSlope.get() * distance + PassingCalculatorTable.kAngleIntercept.get();
    }
}