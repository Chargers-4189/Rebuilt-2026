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

  public enum ShootingType {
    SCORE,
    STATIC,
    PASS
  }

  /** Creates a new Shooter. */
  public Shooter(Vision vision) {
    this.hood = new Hood();
    this.flywheel = new Flywheel();
    this.vision = vision;
  }

  public void stop() {
    hood.home();
    flywheel.stop();
  }

  public void prespin() {
    hood.home();
    flywheel.setVelocitySimple(AutoTable.kPreSpinVelocity.get());
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

  public void align(ShootingType shootingType) {
    switch (shootingType) {
      case SCORE:
        scoreAlign();
        break;
      case STATIC:
        staticAlign();
        break;
      case PASS:
        passAlign();
        break;
    }
  }

  public void scoreAlign() {
    align(vision.getDistanceFromHub(), true);
  }

  public void staticAlign() {
    align(FlywheelTable.kFixedShootDistance.get(), true);
  }

  public void passAlign() {
    align(vision.getDistanceToOurZone(), false);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
