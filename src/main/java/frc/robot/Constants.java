// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
  public static class IndexerConstants {
    public static final double kSpeed = 0.1; //THIS IS A PLACEHOLDER
    public static final int kMotorCanID = 25;
  }

  public static class ShooterConstants {
    public static final int kMotorCanID = 28;
  }
  public final class IntakeConstants {
    public static final int kIntakeMotor = 0; //Filler
    public static final int kIntakeAxisMotor = 0; //Filler
    public static final int kIntakeEncoder = 0; //Filler
    
    public static final double kIntakeAxisSpeed = 0.5; //Change Later
    public static final double kIntakeAxisOuterLimit = 0.0; //Change Later
    public static final double kIntakeAxisInnerLimit = 0.0; //Change Later
    
    public static final double kIntakeSpeed = 1;

    public static final int kTauntAmount = 3; //This is how many times it goes up and down
    public static final int kTauntFraction = 3; //The fraction of how far it goes into the bot
  }
  public static class HopperConstants {

    //Subsystem
    public static final int kMOTOR_ID_RIGHT = 0; //Filler ID's
    public static final int kMOTOR_ID_LEFT = 0; //Filler ID's
    
  }
}
