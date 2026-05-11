// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FlywheelConstants;
import frc.robot.util.NetworkTables.FlywheelTable;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */

  //Flywheel motor shoots balls upwards from the hopper and indexer motor
  private final TalonFXS leftFlywheelMotor = new TalonFXS( 
    FlywheelConstants.kLeftMotorCanID 
  );
  private final TalonFXS rightFlywheelMotor = new TalonFXS( 
    FlywheelConstants.kRightMotorCanID 
  );

  private TalonFXSConfiguration talonFXSConfigs;
  private Slot0Configs slot0Configs;
  private MotionMagicConfigs motionMagicConfigs;
  private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  private double targetVelocity;
 
  public Flywheel() {
    configureMotors();
  }

  public void configureMotors() {
    talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

    // set slot 0 gainss
    slot0Configs = talonFXSConfigs.Slot0;
    slot0Configs.kS = FlywheelTable.kS.get(); // Add 0.25 V output to overcome static friction
    slot0Configs.kV = FlywheelTable.kV.get(); // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = FlywheelTable.kA.get(); // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = FlywheelTable.kP.get(); // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = FlywheelTable.kI.get(); // no output for integrated error
    slot0Configs.kD = FlywheelTable.kD.get(); // no output for error derivative

    // set Motion Magic settings
    motionMagicConfigs = talonFXSConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = FlywheelTable.kMotionMagicAcceleration.get(); // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = FlywheelTable.kMotionMagicJerk.get(); // Target jerk of 4000 rps/s/s (0.1 seconds)

    leftFlywheelMotor.getConfigurator().apply(talonFXSConfigs);
    rightFlywheelMotor.getConfigurator().apply(talonFXSConfigs);
  }

  public void setPower(double power) {
    leftFlywheelMotor.set(power);
    rightFlywheelMotor.set(-power);
  }

  public void stop() {
    setPower(0);
  }

  public double getVelocity() {
    return -rightFlywheelMotor.getVelocity().getValueAsDouble();
  }

  public void setVelocitySimple(double velocity) {
    targetVelocity = velocity;
    leftFlywheelMotor.setControl(m_request.withVelocity(velocity));
    rightFlywheelMotor.setControl(m_request.withVelocity(-velocity));
  }

  public void setVelocity(double velocity) {
    if (getVelocity() < velocity - FlywheelTable.kMaxPowerCutoff.get()) {
      targetVelocity = velocity;
      if (velocity > 5) { //Just in case of a wierd coding error, prevents the flywheel from moving when it shouldn't.
        setPower(FlywheelTable.kSuperSpinPower.get());
      }
    } else {
      setVelocitySimple(velocity);
    }
  }

  public Command setVelocityCommand(DoubleSupplier velocity) {
    return Commands.runEnd(
      () -> setVelocity(velocity.getAsDouble()),
      () -> stop(),
      this
    ).withName(
      "Set Flywheel Velocity"
    );
  }

  public boolean isAligned() {
    return Math.abs(getVelocity() - targetVelocity) < FlywheelTable.kTolerance.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    FlywheelTable.velocity.set(getVelocity());
    FlywheelTable.velocityGoal.set(targetVelocity);
    FlywheelTable.currentPower.set(leftFlywheelMotor.getMotorVoltage().getValueAsDouble() / 12.0);
  }
}
