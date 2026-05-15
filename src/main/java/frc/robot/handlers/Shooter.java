// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.util.NetworkTables.AutoTable;
import frc.robot.util.NetworkTables.FlywheelTable;
import frc.robot.util.NetworkTables.PassingCalculatorTable;
import frc.robot.util.NetworkTables.ShootingCalculatorTable;

public class Shooter extends SubsystemBase {

  /** Hood Subsystem - Fully controlled by the Shooter.*/
  private final Hood hood;
  /** Flywheel Subsystem - Fully controlled by the Shooter.*/
  private final Flywheel flywheel;
  /** Vision Subsystem */
  private final Vision vision;
  
  /** Current State of the Shooter.*/
  private ShooterState shooterState = ShooterState.STOPPED;

  /** Specifies whether the Shooter is aligned. */
  private boolean isAligned = false;

  /** An enum for representing states the shooter could be in. */
  public enum ShooterState {
    SCORING,
    STATIC_SHOOTING,
    PASSING,
    STOPPED,
    PRESPIN
  }

  /** Creates a new Shooter. */
  public Shooter(Vision vision) {
    this.hood = new Hood();
    this.flywheel = new Flywheel();
    this.vision = vision;
  }

  public void setState(ShooterState shooterState) {
    this.shooterState = shooterState;
  }

  public void align(double distance, boolean scoring) {
    if (scoring) {
      hood.setAngle(calculateScoringAngle(distance));
      flywheel.setVelocity(calculateScoringPower(distance));
    } else {
      hood.setAngle(calculatePassingAngle(distance));
      flywheel.setVelocity(calculatePassingPower(distance));
    }
  }

  private static double calculateScoringAngle(double distance) {
      return ShootingCalculatorTable.kAngleSlope.get() * distance + ShootingCalculatorTable.kAngleIntercept.get();
  }

  private static double calculateScoringPower(double distance) {
      return ShootingCalculatorTable.kVelocitySquared.get() * distance * distance + ShootingCalculatorTable.kVelocitySlope.get() * distance + ShootingCalculatorTable.kVelocityIntercept.get();
  }

  private static double calculatePassingPower(double distance) {
      return MathUtil.clamp(
          ShootingCalculatorTable.kVelocitySquared.get() * distance * distance + PassingCalculatorTable.kVelocitySlope.get() * distance + PassingCalculatorTable.kVelocityIntercept.get(),
          PassingCalculatorTable.kMinVelocity.get(),
          PassingCalculatorTable.kMaxVelocity.get()
      );
  }

  private static double calculatePassingAngle(double distance) {
      return MathUtil.clamp(
          PassingCalculatorTable.kAngleSlope.get() * distance + PassingCalculatorTable.kAngleIntercept.get(),
          PassingCalculatorTable.kMinHoodAngle.get(),
          PassingCalculatorTable.kMaxHoodAngle.get()
      );
  }

  public boolean isAligned() {
    return isAligned;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (shooterState) {
      case STOPPED:
        hood.home();
        flywheel.stop();
        isAligned = false;
        break;
      case SCORING:
        align(vision.getDistanceFromHub(), true);
        if (flywheel.isAligned()) {
          isAligned = true;
        }
        break;
      case STATIC_SHOOTING:
        align(FlywheelTable.kFixedShootDistance.get(), true);
        if (flywheel.isAligned()) {
          isAligned = true;
        }
        break;
      case PASSING:
        align(vision.getDistanceToOurZone(), false);
        if (flywheel.isAligned()) {
          isAligned = true;
        }
        break;
      case PRESPIN:
        hood.home();
        flywheel.setVelocitySimple(AutoTable.kPreSpinVelocity.get());
        isAligned = false;
        break;
    }
  }
}
