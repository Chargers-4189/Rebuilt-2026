// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.NetworkTables;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveIndexer extends Command {
  /** Creates a new Shoot. */
  private Indexer indexer;
  private Shooter shooter;
  public MoveIndexer(Indexer indexer, Shooter shooter) {
    this.indexer = indexer;
    this.shooter = shooter;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs((shooter.getVelocity() - shooter.getTargetVelocity())) <= ShooterConstants.kTolerance){
      indexer.setIndexerPower(0.4);
    }else{
      indexer.setIndexerPower(-0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      indexer.setIndexerPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
