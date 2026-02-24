// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.NetworkTables.HopperTable;
import frc.robot.util.NetworkTables.IndexerTable;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadFuel extends Command {
  /** Creates a new Shoot. */
  private Indexer indexer;
  private Shooter shooter;
  private SwerveSubsystem swerve;
  private boolean checkAlignment;
  private Hopper hopper;
  private boolean alreadyAligned;
  private boolean alreadyFast;

  public LoadFuel(Indexer indexer, Hopper hopper, Shooter shooter, SwerveSubsystem swerve, boolean checkAlignment) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.swerve = swerve;
    this.checkAlignment = checkAlignment;
    this.hopper = hopper;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alreadyAligned = false;
    alreadyFast = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!checkAlignment && alreadyAligned) {
      indexer.setIndexerPower(IndexerTable.kPower.get());
      hopper.setSpeed(HopperTable.kPower.get());
    } else {
      if (Math.abs((shooter.getVelocity() - shooter.getTargetVelocity())) >= ShooterConstants.kTolerance){
        System.out.println("Not Enough Power");
        indexer.setIndexerPower(IndexerConstants.kReversePower);
        hopper.setSpeed(0);
      } else if (Math.abs((swerve.getRotations() - swerve.getRotationalGoal())) >= SwerveConstants.kTolerance) {
        System.out.println("Not Rotated Enough");
        indexer.setIndexerPower(IndexerConstants.kReversePower);
        hopper.setSpeed(0);
        
      } else {
        indexer.setIndexerPower(IndexerTable.kPower.get());
        hopper.setSpeed(HopperTable.kPower.get());
        alreadyAligned = true;
      }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      indexer.setIndexerPower(0);
      hopper.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
