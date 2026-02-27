// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.NetworkTables.HoodTable;
import frc.robot.util.OffsetEncoder;
import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase {

  private final TalonFXS hoodMotor = new TalonFXS(
    HoodConstants.kMotorCanID
  );

  private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(0);
  private OffsetEncoder offsetEncoder = new OffsetEncoder(0, .675, hoodEncoder::get);

  private final PIDController hoodController = new PIDController(
    HoodTable.kP.get(),
    HoodTable.kI.get(),
    HoodTable.kD.get()
  );

  public Hood() {
    hoodEncoder.setInverted(true);
  }

  public void setPower(double power) {
    hoodMotor.set(-power);
  }

  public double getHoodPosition() {
    return hoodEncoder.get();
  }

  public void zeroEncoder() {
    System.out.println("ERROR: Use Rev Software to reset this.");
  }
  
  public Command setHoodAngleCommand(DoubleSupplier angle) {
    return Commands.run(
        () -> {
          setHoodAngle(angle.getAsDouble());
        }, this).withName("HoodAlign");
  }

  public void setHoodAngle(double angle) {
    HoodTable.hoodGoal.set(angle);
    hoodMotor.set(
      -MathUtil.clamp(hoodController.calculate(
        offsetEncoder.get(),
        offsetEncoder.convertGoal(angle)
      ),
      -HoodConstants.kAutoPower,
      HoodConstants.kAutoPower)
    );
  }

  public void setHoodAngle(DoubleSupplier angle) {
    setHoodAngle(angle.getAsDouble());
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /*
    System.out.print("Connected: " + hoodEncoder.isConnected() + " ");
    System.out.print("Raw Encoder: " + hoodEncoder.get() + " ");
    System.out.print("Encoder: " + getHoodPosition() + " ");
    System.out.println();
    */

    HoodTable.hoodEncoder.set(getHoodPosition());
    
    hoodController.setPID(
      HoodTable.kP.get(),
      HoodTable.kI.get(),
      HoodTable.kD.get()
    );
  }
}