// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.NetworkTables.HopperTable;
import frc.robot.util.NetworkTables.IndexerTable;
import frc.robot.util.NetworkTables.ShooterTable;
import frc.robot.util.NetworkTables.SwerveTable;
import frc.robot.util.Stopwatch;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadFuel extends Command {
  private Indexer indexer;
  private Shooter shooter;
  private SwerveSubsystem swerve;
  private Hopper hopper;
  private boolean alreadyAligned;
  private boolean swerveRotateNeeded;

  private Stopwatch stopwatch = new Stopwatch();

  /** Creates a new LoadFuel. */
  public LoadFuel(Indexer indexer, Hopper hopper, Shooter shooter, SwerveSubsystem swerve, boolean swerveRotateNeeded) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.swerve = swerve;
    this.hopper = hopper;
    this.swerveRotateNeeded = swerveRotateNeeded;
    addRequirements(indexer, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alreadyAligned = false;
    stopwatch.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    checkSwerveRotation();
    if (alreadyAligned) {
      System.out.println("Loading Fuel!");
      indexer.setPower(IndexerTable.kPower.get());
      hopper.setPower(HopperTable.kPower.get());
    } else {
      if (Math.abs(shooter.getVelocity() - shooter.getTargetVelocity()) > ShooterTable.kTolerance.get()){
        System.out.println("Not Enough Power");
        indexer.setPower(IndexerConstants.kReversePower);
        hopper.setPower(0);
      } else if (swerveRotateNeeded && !swerveRotated()) {
        System.out.println("Not Rotated Enough");
        indexer.setPower(IndexerConstants.kReversePower);
        hopper.setPower(0); 
      } else {
        System.out.println("Loading Fuel!");
        indexer.setPower(IndexerTable.kPower.get());
        hopper.setPower(HopperTable.kPower.get());
        alreadyAligned = true;
      }
    }
  }

  private boolean swerveRotated() {
    return stopwatch.hasStarted() && stopwatch.hasTriggered();
  }

  private void checkSwerveRotation() {
    if (!stopwatch.hasStarted() && Math.abs(swerve.getRotationalError()) <= SwerveTable.kAngleTolerance.get()) {
      stopwatch.start(1000 * SwerveTable.kExtraRotationSeconds.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      indexer.setPower(0);
      hopper.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
