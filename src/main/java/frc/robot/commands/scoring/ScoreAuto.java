// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAuto extends ParallelCommandGroup {
  
  /** Creates a new ScoreAuto. */
  public ScoreAuto(Shooter shooter, Hood hood, Indexer indexer, SwerveSubsystem swerve, Vision vision, Hopper hopper) {
    addCommands(
        new SequentialCommandGroup(Commands.waitSeconds(.5), new LoadFuel(indexer, hopper, shooter, swerve, false)),
        new AlignHoodAndFlywheel(hood, shooter, vision),
        new AlignSwerve(swerve, vision, () -> 0, () -> 0)
    );
  }
}

