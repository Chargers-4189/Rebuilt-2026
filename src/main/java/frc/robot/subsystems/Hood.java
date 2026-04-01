// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.NetworkTables.HoodTable;
import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase {

  private final TalonFXS hoodMotor = new TalonFXS(
    HoodConstants.kMotorCanID
  );

  private final CANcoder hoodEncoder = new CANcoder(HoodConstants.kEncoderID);

  private final PIDController hoodController = new PIDController(
    HoodTable.kP.get(),
    HoodTable.kI.get(),
    HoodTable.kD.get()
  );

  public Hood() {
    TalonFXSConfiguration talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    talonFXSConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodMotor.getConfigurator().apply(talonFXSConfigs);
    
    hoodEncoder.getConfigurator().apply(HoodConstants.kHoodEncoderConfigs);
  }

  public void setPower(double power) {
    hoodMotor.set(-power);
  }

  public double getHoodPosition() {
    return hoodEncoder.getAbsolutePosition().getValueAsDouble();
  }
  
  public Command setHoodAngleCommand(DoubleSupplier angle) {
    return Commands.run(() -> setHoodAngle(angle.getAsDouble()), this)
        .finallyDo(() -> setPower(0))
        .withName("HoodAlign");
  }

  public void setHoodAngle(double angle) {
    HoodTable.hoodGoal.set(angle);
    if (hoodEncoder.isConnected()) {
      hoodMotor.set(
        -MathUtil.clamp(hoodController.calculate(
          getHoodPosition(),
          angle
        ),
        -HoodConstants.kAutoPower,
        HoodConstants.kAutoPower)
      );
    } else {
      setPower(0);
      System.out.println("ERROR: Hood Encoder Disconnected");
    }
  }

  public void setHoodAngle(DoubleSupplier angle) {
    setHoodAngle(angle.getAsDouble());
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    HoodTable.hoodEncoder.set(getHoodPosition());
    
    hoodController.setPID(
      HoodTable.kP.get(),
      HoodTable.kI.get(),
      HoodTable.kD.get()
    );
  }
}