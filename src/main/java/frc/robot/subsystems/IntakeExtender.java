// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NetworkTables.IntakeTable;

public class IntakeExtender extends SubsystemBase {

  /** IntakeExtender Motor */
  private TalonFXS extenderMotor = new TalonFXS(Constants.IntakeConstants.kExtenderMotor);

  /** IntakeExtender Encoder */
  private CANcoder extenderEncoder = new CANcoder(Constants.IntakeConstants.kIntakeEncoder);

  /** PID Controller for the IntakeExtender. */
  private ProfiledPIDController pidController;

  /** Stores the most recent angle setpoint applied to the IntakeExtender.*/
  private double extensionGoal = 0;

  /** Creates a new IntakeExtender. */
  public IntakeExtender() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    talonFXSConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extenderMotor.getConfigurator().apply(talonFXSConfigs);

    pidController = new ProfiledPIDController(
      IntakeTable.kP.get(),
      IntakeTable.kI.get(),
      IntakeTable.kD.get(),
      new TrapezoidProfile.Constraints(IntakeTable.kMaxVelocity.get(), IntakeTable.kMaxAcceleration.get())
    );
    pidController.setTolerance(IntakeTable.kTolerance.get());
  }

  /**
   * Sets the extender motor to the given power. Positive is retracting the intake.
   * Zero is fully extended.
   * 
   * @param power the power to run the extender.
   */
  public void setPower(double power) {
    extenderMotor.set(-power);
  }

  /** Deactivates the extender motor. */
  public void stop() {
    setPower(0);
  }

  /**
   * Gets the current angle of the extender via the hood encoder. Positive is retracting
   * the intake. Zero is fully extended.
   * 
   * @return the current angle (rotations)
   */
  public double getPosition() {
    return extenderEncoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Returns true if the extender encoder is connected.
   * 
   * @return whether the encoder is connected.
   */
  public boolean isEncoderConnected() {
    return extenderEncoder.isConnected();
  }

  /**
   * Sets the extender to the given angle via PID. Positive is retracting
   * the intake. Zero is fully extended.
   * 
   * @param angle the new angle (rotations)
   */
  public void setAngle(double angle) {
    extensionGoal = angle;

    if (isEncoderConnected()) {
      setPower(
        MathUtil.clamp(calculatePIDS(
          getPosition(),
          angle
        ),
        -IntakeTable.kAutoInPower.get(),
        IntakeTable.kAutoOutPower.get())
      );
    } else {
      stop();
      System.out.println("ERROR: Intake Encoder Disconnected");
    }
  }

  /**
   * Calculates the new PIDS result given the current measurement and the goal.
   *
   * @param measurement the current angle of the intake extender
   * @param goal the target angle of the intake extender
   * @return the power to run the extender motor
   */
  private double calculatePIDS(double measurement, double goal) {
    double pidCalculation = pidController.calculate(measurement, goal);
    return pidCalculation + Math.copySign(IntakeTable.kS.get(), pidCalculation);
  }

  /**
   * Returns true if the exender is at its target angle.
   *
   * @return whether the extender is at its target angle.
   */
  public boolean isAligned() {
    return pidController.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    IntakeTable.extensionGoal.set(extensionGoal);
    IntakeTable.rawEncoder.set(getPosition());
  }
}
