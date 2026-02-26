// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopAll extends Command {
  Hood hood;
  Hopper hopper;
  Indexer indexer;
  Intake intake;
  Shooter shooter;
  SwerveSubsystem swerve;

  SwerveRequest brake = new SwerveRequest.SwerveDriveBrake();
  /** Creates a new CancelAll. */
  public StopAll(Hood hood, Hopper hopper, Indexer indexer, Intake intake, Shooter shooter, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood, hopper, indexer, intake, shooter, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setPower(0);
    hopper.setPower(0);
    indexer.setPower(0);
    intake.setExtensionPower(0);
    intake.setWheelPower(0);
    shooter.setVelocity(0);
    swerve.applyRequest(() -> brake);
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
