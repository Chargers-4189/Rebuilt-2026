// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables.AutoTable;
import frc.robot.util.NetworkTables.FlywheelTable;
import frc.robot.util.NetworkTables.PassingCalculatorTable;
import frc.robot.util.NetworkTables.ShootingCalculatorTable;

public class Shooter extends SubsystemBase {

  public final Hood hood;
  public final Flywheel flywheel;
  public final Vision vision;
  
  private ShooterState shooterState = ShooterState.STOPPED;
  private boolean isAligned = false;

  public enum ShootingType {
    SCORE,
    STATIC,
    PASS
  }

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
