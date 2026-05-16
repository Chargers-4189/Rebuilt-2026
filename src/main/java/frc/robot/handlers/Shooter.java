// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Vision;
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
  private ShooterState shooterState = ShooterState.DEFAULT;

  /** Specifies whether the Shooter is aligned. */
  private boolean isAligned = false;

  /** An enum for representing states the shooter could be in. */
  public enum ShooterState {
    /** Scoring in the hub. */
    SCORING,
    /** Fixed distance scoring in the hub. */
    STATIC_SHOOTING,
    /** Passing fuel to our alliance zone. */
    PASSING,
    /** Stopping the flywheel & homing the hood. */
    DEFAULT,
    /** Spinning up the shooter to fixed velocity. */
    PRESPIN,
    /** Deactivating all motors */
    STOPPED
  }

  /** Creates a new Shooter. */
  public Shooter(Vision vision) {
    this.hood = new Hood();
    this.flywheel = new Flywheel();
    this.vision = vision;
  }

  /**
   * Sets the state of the shooter to the given state.
   * 
   * @param shooterState the new state
   */
  public void setState(ShooterState shooterState) {
    this.shooterState = shooterState;
  }

  /**
   * Aligns the hood and spins the flywheel to the appropriate angle and velocity.
   *
   * @param distance the distance between the robot and the target
   * @param scoring whether the scoring formula should be used instead of the passing formula
   */
  public void align(double distance, boolean scoring) {
    if (scoring) {
      hood.setAngle(calculateScoringAngle(distance));
      flywheel.setVelocity(calculateScoringVelocity(distance));
    } else {
      hood.setAngle(calculatePassingAngle(distance));
      flywheel.setVelocity(calculatePassingVelocity(distance));
    }
  }

  /**
   * Calculates the appropriate hood angle to score from the given distance.
   *
   * @param distance the distance to the hub
   * @return the hood angle
   */
  private static double calculateScoringAngle(double distance) {
      return ShootingCalculatorTable.kAngleSlope.get() * distance + ShootingCalculatorTable.kAngleIntercept.get();
  }

  /**
   * Calculates the appropriate flywheel velocity to score from the given distance.
   *
   * @param distance the distance to the hub
   * @return the flywheel velocithy
   */
  private static double calculateScoringVelocity(double distance) {
      return ShootingCalculatorTable.kVelocitySquared.get() * distance * distance + ShootingCalculatorTable.kVelocitySlope.get() * distance + ShootingCalculatorTable.kVelocityIntercept.get();
  }

  /**
   * Calculates the appropriate hood angle to pass from the given distance.
   *
   * @param distance the distance to target
   * @return the hood angle
   */
  private static double calculatePassingVelocity(double distance) {
      return MathUtil.clamp(
          ShootingCalculatorTable.kVelocitySquared.get() * distance * distance + PassingCalculatorTable.kVelocitySlope.get() * distance + PassingCalculatorTable.kVelocityIntercept.get(),
          PassingCalculatorTable.kMinVelocity.get(),
          PassingCalculatorTable.kMaxVelocity.get()
      );
  }

  /**
   * Calculates the appropriate flywheel velocity
   * to pass from the given distance.
   *
   * @param distance the distance to target
   * @return the flywheel velocity
   */
  private static double calculatePassingAngle(double distance) {
      return MathUtil.clamp(
          PassingCalculatorTable.kAngleSlope.get() * distance + PassingCalculatorTable.kAngleIntercept.get(),
          PassingCalculatorTable.kMinHoodAngle.get(),
          PassingCalculatorTable.kMaxHoodAngle.get()
      );
  }

  /**
   * Returns true if the shooter is aligned & ready to shoot.
   *
   * @return whether the shooter is aligned
   */
  public boolean isAligned() {
    return isAligned;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (shooterState) {
      case DEFAULT:
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
      case STOPPED:
        hood.stop();
        flywheel.stop();
        isAligned = false;
    }
  }
}
