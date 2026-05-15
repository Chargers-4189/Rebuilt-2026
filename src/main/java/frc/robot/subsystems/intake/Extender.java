// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NetworkTables.IntakeTable;

public class Extender extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS extensionMotor = new TalonFXS(Constants.IntakeConstants.kExtenderMotor);
  private CANcoder encoder = new CANcoder(Constants.IntakeConstants.kIntakeEncoder);

  private ProfiledPIDController pidController;
  private double goal = 0;

  public Extender() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    talonFXSConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionMotor.getConfigurator().apply(talonFXSConfigs);

    pidController = new ProfiledPIDController(
      IntakeTable.kP.get(),
      IntakeTable.kI.get(),
      IntakeTable.kD.get(),
      new TrapezoidProfile.Constraints(IntakeTable.kMaxVelocity.get(), IntakeTable.kMaxAcceleration.get())
    );
    pidController.setTolerance(IntakeTable.kTolerance.get());
  }

  //+: Rotates Clockwise (Out), -: Rotates Counterclockwise (In)
  public void setExtensionPower(double power) {
    extensionMotor.set(-power);
  }

  public void stop() {
    setExtensionPower(0);
  }

  public Command setPowerCommand(DoubleSupplier power) {
    return Commands.runEnd(
      () -> setExtensionPower(power.getAsDouble()), 
      () -> stop(),
      this
    ).withName(
      "Set Intake Extension Power"
    );
  }

  public double getEncoder() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean encoderConnected() {
    return encoder.isConnected();
  }

  public void setAngle(double angle) {
    goal = angle;

    if (encoderConnected()) {
      setExtensionPower(
        MathUtil.clamp(calculatePIDS(
          getEncoder(),
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

  private double calculatePIDS(double measurement, double goal) {
    double pidCalculation = pidController.calculate(measurement, goal);
    return pidCalculation + Math.copySign(IntakeTable.kS.get(), pidCalculation);
  }

  public boolean isAligned() {
    return pidController.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    IntakeTable.extensionGoal.set(goal);
    IntakeTable.rawEncoder.set(getEncoder());
  }
}
