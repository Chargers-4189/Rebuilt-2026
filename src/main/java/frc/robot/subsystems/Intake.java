// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.revrobotics.spark.config.FeedForwardConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.NetworkTables.HoodTable;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.util.OffsetEncoder;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFXS wheelMotor = new TalonFXS(Constants.IntakeConstants.kIntakeMotor); //Needs to be inverted
  private TalonFXS extensionMotor = new TalonFXS(Constants.IntakeConstants.kIntakeAxisMotor);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.IntakeConstants.kIntakeEncoder);

  private OffsetEncoder offsetEncoder = new OffsetEncoder(.415, .849);

  private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.18399, 0);

  private final PIDController intakeController = new PIDController(
    IntakeTable.kP.get(),
    IntakeTable.kI.get(),
    IntakeTable.kD.get()
  );

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(.3).per(Second),        // Use default ramp rate (1 V/s)
         Volts.of(3), // Reduce dynamic step voltage to 4 to prevent brownout
         Seconds.of(15),        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("stateV2", state.toString())
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
    wheelMotor.set(-speed); //Change once inverted
  }

  //+: Rotates Clockwise (Out), -: Rotates Counterclockwise (In)
  public void setExtensionSpeed(double speed) {
    extensionMotor.set(speed);
  }

  public void setExtensionAngle(double angle) {
    IntakeTable.extensionGoal.set(angle);
    extensionMotor.set(
      MathUtil.clamp(intakeController.calculate(
        offsetEncoder.convertCurrent(getEncoder()),
        offsetEncoder.convertGoal(angle)
      ),
      -IntakeTable.kExtensionMaxPower.get(),
      IntakeTable.kExtensionMaxPower.get())
    );
  }

  public double getEncoder() {
    return encoder.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getEncoder());

    intakeController.setPID(
      IntakeTable.kP.get(),
      IntakeTable.kI.get(),
      IntakeTable.kD.get()
    );
  }
}
