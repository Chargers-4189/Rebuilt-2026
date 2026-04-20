// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.NetworkTables.SwerveTable;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignAngle extends Command {

  private SwerveSubsystem swerve;

  private DoubleSupplier xPower;
  private DoubleSupplier yPower;

  private DoubleSupplier rotationGoal;

  private PIDController xPid = new PIDController(0, 0, 0);
  private PIDController yPid = new PIDController(0, 0, 0);
  private PIDController anglePid = new PIDController(0, 0, 0);

  private boolean biDirectional;

  private double anglePidCalculation;
  private double kSCalculation;

  private final SwerveRequest.FieldCentric drive;

  /** Creates a new AutoAlignPose. */
  public AlignAngle(SwerveSubsystem swerve, DoubleSupplier xPower, DoubleSupplier yPower, DoubleSupplier rotationGoal, boolean biDirectional) {
    this.swerve = swerve;
    this.xPower = xPower;
    this.yPower = yPower;
    this.rotationGoal = rotationGoal;
    this.biDirectional = biDirectional;
    this.drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveSubsystem.MaxSpeed * 0.1).withRotationalDeadband(SwerveSubsystem.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  public AlignAngle(SwerveSubsystem swerve, CommandXboxController controller, DoubleSupplier rotationGoal, boolean biDirectional) {
    this(swerve, () -> -controller.getLeftY(), () -> -controller.getLeftX(), rotationGoal, biDirectional);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePid.enableContinuousInput(-.5, .5);

    xPid.setPID(SwerveTable.kPositionP.get(), SwerveTable.kPositionI.get(), SwerveTable.kPositionD.get());
    yPid.setPID(SwerveTable.kPositionP.get(), SwerveTable.kPositionI.get(), SwerveTable.kPositionD.get());
    anglePid.setPID(SwerveTable.kAngleP.get(), SwerveTable.kAngleI.get(), SwerveTable.kAngleD.get());

    xPid.setTolerance(SwerveTable.kPositionTolerance.get());
    yPid.setTolerance(SwerveTable.kPositionTolerance.get());
    anglePid.setTolerance(SwerveTable.kAngleTolerance.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    anglePid.setSetpoint(rotationGoal.getAsDouble());
    double error1 =anglePid.getError();
    double setpoint1 = anglePid.getSetpoint();
    
    if (biDirectional && Math.abs(anglePid.getError()) > .25) {
      anglePid.setSetpoint(rotationGoal.getAsDouble() + .5);
    }
    double error2 = anglePid.getError();
    double setpoint2 = anglePid.getSetpoint();

    System.out.println(String.format("Error1: %.3f Set1: %.3f Error2: %.3f Set2: %.3f", error1, setpoint1, error2, setpoint2));

    anglePidCalculation = anglePid.calculate(swerve.getRotations());
    kSCalculation = Math.copySign(SwerveTable.kAngleS.get(), anglePid.getError());

    double anglePower = MathUtil.clamp(
      anglePidCalculation + kSCalculation,
      -SwerveTable.kAngleMaxPower.get(),
      SwerveTable.kAngleMaxPower.get()
    );

    swerve.setControl(
      drive.withVelocityX(MathUtil.copyDirectionPow(xPower.getAsDouble(), SwerveTable.kDriveExponent.get()) * SwerveSubsystem.MaxSpeed)
           .withVelocityY(MathUtil.copyDirectionPow(yPower.getAsDouble(), SwerveTable.kDriveExponent.get()) * SwerveSubsystem.MaxSpeed)
           .withRotationalRate(anglePower * SwerveSubsystem.MaxAngularRate)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setControl(SwerveSubsystem.idle);
  }

  // Returns true when the command should end.P
  @Override
  public boolean isFinished() {
    return anglePid.atSetpoint() && xPid.atSetpoint() && yPid.atSetpoint();
  }
}
