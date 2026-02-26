// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.util.OffsetEncoder;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS wheelMotor = new TalonFXS(Constants.IntakeConstants.kIntakeMotor); //Needs to be inverted
  private TalonFXS extensionMotor = new TalonFXS(Constants.IntakeConstants.kIntakeAxisMotor);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.IntakeConstants.kIntakeEncoder);

  private OffsetEncoder offsetEncoder = new OffsetEncoder(.415, .849, encoder::get);

  private final PIDController intakeController = new PIDController(
    IntakeTable.kP.get(),
    IntakeTable.kI.get(),
    IntakeTable.kD.get()
  );

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(.3).per(Second), // Use default ramp rate (1 V/s)
         Volts.of(3), // Reduce dynamic step voltage to 4 to prevent brownout
         Seconds.of(15), // Use default timeout (10 s)
         (state) -> SignalLogger.writeString("stateV2", state.toString()) // Log state with Phoenix SignalLogger class
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> extensionMotor.setControl(m_voltReq.withOutput(volts)),
         null,
         this
      )
   );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Intake() {}

  //+: Fuel go in robot, -: Fuel go out robot
  public void setWheelSpeed(double speed) {
    wheelMotor.set(-speed);
  }

  //+: Rotates Clockwise (Out), -: Rotates Counterclockwise (In)
  public void setExtensionSpeed(double speed) {
    extensionMotor.set(speed);
  }

  public void setExtensionAngle(double angle) {
    IntakeTable.extensionGoal.set(offsetEncoder.convertGoal(angle));
    extensionMotor.set(
      MathUtil.clamp(intakeController.calculate(
        offsetEncoder.get(),
        offsetEncoder.convertGoal(angle)
      ),
      -IntakeTable.kAutoOutPower.get(),
      IntakeTable.kAutoInPower.get())
    );
  }

  public double getEncoder() {
    return encoder.get();
  }

  public OffsetEncoder getOffsetEncoder() {
    return offsetEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    IntakeTable.encoder.set(getEncoder());

    intakeController.setPID(
      IntakeTable.kP.get(),
      IntakeTable.kI.get(),
      IntakeTable.kD.get()
    );
  }
}
