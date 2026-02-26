// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables.SwerveTable;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignSwerve extends Command {

  SwerveSubsystem swerve;
  Vision vision;
  DoubleSupplier powerX;
  DoubleSupplier powerY;

  private final SwerveRequest.FieldCentricFacingAngle driveWithAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(swerve.MaxSpeed * 0.1).withRotationalDeadband(swerve.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  
  /** Creates a new AlignSwerve. */
  public AlignSwerve(SwerveSubsystem swerve, Vision vision, DoubleSupplier powerX, DoubleSupplier powerY) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setControl(
      driveWithAngle.withVelocityX(powerX.getAsDouble() * swerve.MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(powerY.getAsDouble() * swerve.MaxSpeed) // Drive left with negative X (left)
          .withTargetDirection(vision.getRotationFromHub())
          .withHeadingPID(SwerveTable.kP.get(), SwerveTable.kI.get(), SwerveTable.kD.get())
          .withMaxAbsRotationalRate(swerve.MaxAngularRate * SwerveTable.kMaxPower.get())
          .withTargetRateFeedforward(swerve.calculateFeedForward(vision.getRotationFromHub()))
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
