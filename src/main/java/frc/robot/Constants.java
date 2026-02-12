// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class HoodConstants {
    public static final double kGearRatio = 5.5;
    public static final double kHoodTolerance = -1; //THIS IS A PLACEHOLDER
    public static final double kHoodPower = .05;  //THIS IS A PLACEHOLDER
    public static final int kMotorCanID = 29;
    public static final int kEncoderDIO = 0;
  }
  public static class IndexerConstants {
    public static final double kSpeed = 0.1; //THIS IS A PLACEHOLDER
    public static final int kMotorCanID = 25;
  }

  public static class ShooterConstants {
    public static final int kMotorCanID = 28;
  }

  public final class IntakeConstants {
    public static final int kWheelMotor = 21; //Filler
    public static final int kExtensionMotor = 22;
    public static final int kExtensionEncoder = 1; //Filler
  }
  public static class HopperConstants {

    //Subsystem
    public static final int kMOTOR_ID_RIGHT = 26;
    //public static final int kMOTOR_ID_LEFT = -1;
    
  }
}
