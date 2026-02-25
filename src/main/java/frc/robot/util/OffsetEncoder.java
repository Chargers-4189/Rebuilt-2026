// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class OffsetEncoder {

    private double min;
    private double max;

    public OffsetEncoder(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public double convertCurrent(double value) {
        double offset = (max + min) / 2;
        if (min > max) {
            offset += 0.5;
        }
        return positiveMod((value + offset), 1);
    }

    public double convertGoal(double value) {
        double offset = (max + min) / 2;
        if (min > max) {
            offset += 0.5;
        }
        return positiveMod((value + offset + min), 1);
    }

    public double compare(double current, double goal) {
        return convertGoal(goal) - convertCurrent(current);
    }

    public void setBounds(double min, double max) {
        this.min = min;
        this.max = max;
    }

    /**
     * Calculates a positive modulus; always returns a positive result.
     *
     * @param x the input number
     * @param m the modulus
     * 
     * @return x mod m
     */
    public double positiveMod(double x, double m) {
        double mod = x % m;
        if (mod < 0) {
            mod += m;
        }
        return mod;
    }
}
