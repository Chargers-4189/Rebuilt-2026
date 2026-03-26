// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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

  public static class SwerveConstants {

    //Constants for Angle Alignment
    public static final double kAngleP = 4.0;
    public static final double kAngleI = 0.0;
    public static final double kAngleD = 0.0;
    public static final double kAngleS = 0.0637;
    public static final double kAngleMaxPower = 1;
    public static final double kAngleTolerance = 0.015;

    //Constants for Position Alignment
    public static final double kPositionP = 3.0;
    public static final double kPositionI = 0.0;
    public static final double kPositionD = 0.0;
    public static final double kPositionS = 0.0;
    public static final double kPositionMaxPower = 1;
    public static final double kPositionTolerance = 0.05;

    //Constants for Manual Driving
    public static final double kDriveExponent = 1.4;
    public static final double kRotationalExponent = 1.4;
  }

  public static class IntakeConstants {

    public static final MagnetSensorConfigs kHoodEncoderConfigs = new MagnetSensorConfigs()
      .withAbsoluteSensorDiscontinuityPoint(.75)
      .withMagnetOffset(0)
      .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    //IDs
    public static final int kWheelMotor = 21; 
    public static final int kExtenderMotor = 22;
    public static final int kIntakeEncoder = 32; 

    //Modifiables
    public static final double kWheelPower = 1;
    public static final double kLowWheelPower = .3;
    public static final double kManualExtensionPower = .25;

    public static final double kAutoOutPower = .3; //These two are swapped
    public static final double kAutoInPower = .2;

    public static final double kP = 2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kMaxVelocity = 0;
    public static final double kMaxAcceleration = 0;

    public static final double kTauntRotations = .25;
    public static final double kTauntFrequency = 2;
    public static final double kTauntMagnitude = Math.PI/16; 

    public static final double kTolerance = 0.06;
    public static final double kOuterExtensionLimit = 0.0;
    public static final double kInnerExtensionLimit = 0.35;
    public static final double kEncoderOffset = 0.5;

    public static final boolean reverseEncoder = false;

    public static final double kTauntDelay = 2;
 }
  
  public static class HopperConstants {
    //IDs
    public static final int kMotorRight = 26; 

    //Modifiables
    public static final int kPower = 1;
    public static final double kReversePower = -1;
  }

  public static class IndexerConstants {
    //IDs
    public static final int kMotorCanID = 25;

    //Modifiables
    public static final double kPower = 1;
    public static final double kReversePower = -.1;
  }

  public static class HoodConstants {

    //Encoder Configs
    public static final MagnetSensorConfigs kHoodEncoderConfigs = new MagnetSensorConfigs()
      .withAbsoluteSensorDiscontinuityPoint(.8)
      .withMagnetOffset(-.33333333333333333)
      .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    
    //IDs
    public static final int kMotorCanID = 27;
    public static final int kEncoderID = 31;

    //Fixed
    public static final double kGearRatio = 5.5;

    //Modifiables
    public static final double kManualPower = .1;
    public static final double kAutoPower = .3;
    public static final double kDefaultAngle = 0.05;

    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class ShooterConstants {
    //IDs
    public static final int kLeftMotorCanID = 29;
    public static final int kRightMotorCanID = 28;

    //Modifiables
    public static final double kFixedPower = .5;
    public static final double kFixedShootDistance = 3.2;
    public static final double kPassVelocity = 70;

    public static final double kTolerance = 1.5;

    //slot 0 configs
    public static final double kS = 0.45; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 0.2; // A position error of 2.5 rotations results in 12 V output 
    public static final double kI = 0.0; // no output for integrated error
    public static final double kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    public static final double kMotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    public static final double kMotionMagicAcceleration = 50; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double kMotionMagicJerk = 4000; // Target jerk of 1600 rps/s/s (0.1 seconds)
  }

  public static class ShootingCalculatorConstants {
    public static final double kAngleIntercept = -.07;
    public static final double kAngleSlope = .06;
    public static final double kVelocityIntercept = 36.5;
    public static final double kVelocitySlope = 4.2;
    public static final double kVelocitySquared = 0;
  }

  public static class PassingCalculatorConstants {
    public static final double kAngleIntercept = -.07;
    public static final double kAngleSlope = .06;
    public static final double kVelocityIntercept = 40;
    public static final double kVelocitySlope = 4.2;
    public static final double kVelocitySquared = 0;
  }

  public static class AutoConstants {
    public static final boolean kRightSide = true;
    
    public static final double kPreSpinDuration = 2; //Seconds
    public static final double kPreSpinVelocity = 55; //Rotations per Second

    public static final double kShooterTimeout = 3.5; //Seconds
  }
}
