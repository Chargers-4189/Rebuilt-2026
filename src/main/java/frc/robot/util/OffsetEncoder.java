// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class OffsetEncoder {

    private DutyCycleEncoder encoder;
    private double min;
    private double max;

    public OffsetEncoder(int channel, double min, double max, boolean inverted) {
        this.encoder = new DutyCycleEncoder(channel);
        encoder.setInverted(inverted);
    }
    
    public OffsetEncoder(int channel, double min, double max) {
        this(channel, min, max, false);
    }

    public double getEncoder() {
        double offset = (max + min) / 2;
        if (min > max) {
            offset += 0.5;
        }
        return (encoder.get() + offset) % 1;
    }

    public void setBounds(double min, double max) {
        this.min = min;
        this.max = max;
    }
}
