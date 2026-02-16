// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
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
    public static final int kTestControllerPort = 1;
  }
  public static class HoodConstants {
    public static final double kGearRatio = 5.5;
    public static final double kHoodTolerance = .02; //THIS IS A PLACEHOLDER
    public static final double kHoodPower = .1;  //THIS IS A PLACEHOLDER
    public static final int kMotorCanID = 29;
    public static final int kEncoderDIO = 0;

    public static final double kANGLE = .915;
    public static final double kP = 1; //All these are tested and stuff
    public static final double kI = 0;
    public static final double kD = 0;
    
  }
  public static class IndexerConstants {
    public static final int kMotorCanID = 25;
  }

  public static class ShooterConstants {
    public static final int kLeftMotorCanID = 27;
    public static final int kRightMotorCanID = 28;

    public static final double kTolerance = 2;

    public static final double kTEST_POWER = .6;
    public static final double kDISTANCE = 3;
    public static final double kFUEL_NUM = 1.5;

    public static final double kSPEED = 0;

    //slot 0 configs
    public static final double kS = 0.83; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.115; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 0.1; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0.0; // no output for integrated error
    public static final double kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

  // set Motion Magic settings
  public static final double MotionMagicCruiseVelocity = 1.0; // Target cruise velocity of 80 rps
  public static final double MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
  public static final double MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

  }
  
  public static class IntakeConstants {
    public static final int kIntakeMotor = 21; 
    public static final int kIntakeAxisMotor = 22;
    public static final int kIntakeEncoder = 1; 
  }

  public static class HopperConstants {
    //Subsystem
    public static final int kHopperMotorRight = 26; 
    public static final int kHopperMotorLeft = 0; //Filler ID
  }
}
