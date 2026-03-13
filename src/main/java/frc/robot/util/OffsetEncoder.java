// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

/**
 * This class is intended to be used for a rotational mechanism with the following characteristics:
 * - It has an absolute encoder 
 * - It's range of motion is limitedsuch that its total rotational capability is less 1 rotation (360 degrees) total
 *
 * The subsystem returns values offset from the encoder such that the discontinuity in the encoder values (where the encoder rotation jumps between 0 and 1)
 * is located at the furthest point in the rotation outside of its range of motion. This allows for a non-continuous PID loop to be effective. This is especially
 * useful for systems that that can rotate more than a half rotation (180 degrees) because the built-in continuous encoder tries to take the shortest route, telling
 * the system to rotate outside of its range of motion.
*/
public class OffsetEncoder {

    private double min;
    private double max;
    private double discontinuityPoint;
    private DoubleSupplier rawEncoder;

    /**
     * Creates a new offset encoder
     *
     * @param min the minimum allowed encoder value (rotations)
     * @param max the maximum allowed encoder value (rotations)
     * @param rawEncoder a supplier that returns an absolute encoder value (rotations)
     */
    public OffsetEncoder(double min, double max, DoubleSupplier rawEncoder) {
        this.min = min;
        this.max = max;
        this.rawEncoder = rawEncoder;
        updateDiscontinuityPoint();
    }

    /**
     * Returns the encoder value to be plugged into a PID calculation
     * @return the encoder value to be plugged into a PID calculation
     */
    public double getFromDiscontinuity() {
        return MathUtil.inputModulus(rawEncoder.getAsDouble() - discontinuityPoint, 0, 1);
    }
    /**
     * Returns the encoder value measured from the minimum
     * @return the encoder value measured from the minimum
     */
    public double getFromMin() {
        return MathUtil.inputModulus(rawEncoder.getAsDouble() - discontinuityPoint + min, 0, 1);
    }

    /**
     * Converts the goal value to be plugged into a PID calculation
     * @return the goal value to be plugged into a PID calculation (rotations)
     */
    public double convertGoal(double value) {
        return MathUtil.inputModulus(value - discontinuityPoint + min, 0, 1);
    }

    /**
     * Calculates the difference between the goal and the current position (goal - get). Useful for end conditions.
     * 
     * @param goal the goal value (rotations)
     * @return the difference between the goal and the current position (goal - get)
     */
    public double compare(double goal) {
        return convertGoal(goal) - getFromDiscontinuity();
    }

    /**
     * Changes the minimum and maximum allowed encoder values.
     *
     * @param min minimum encoder value (rotations)
     * @param max maximum encoder value (rotations)
     */
    public void setBounds(double min, double max) {
        this.min = min;
        this.max = max;
        updateDiscontinuityPoint();
    }

    private void updateDiscontinuityPoint() {
        discontinuityPoint = (max + min) / 2;
        if (min < max) {
            discontinuityPoint += 0.5;
        }
    }
}
