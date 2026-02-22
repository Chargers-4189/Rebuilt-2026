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

    public double calculate(double value) {
        double offset = (max + min) / 2;
        if (min > max) {
            offset += 0.5;
        }
        return (value - offset) % 1;
    }

    public void setBounds(double min, double max) {
        this.min = min;
        this.max = max;
    }
}
