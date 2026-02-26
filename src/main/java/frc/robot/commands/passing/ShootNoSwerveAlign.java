// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.passing;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.scoring.LoadFuel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoSwerveAlign extends ParallelCommandGroup {
  
  /** Creates a new ShootNoSwerveAlign. */
  public ShootNoSwerveAlign(Shooter shooter, Hood hood, Indexer indexer, Hopper hopper, SwerveSubsystem swerve, DoubleEntry speed, double angle) {
    addCommands(
        new SequentialCommandGroup(Commands.waitSeconds(.5), new LoadFuel(indexer, hopper, shooter, swerve, false)),
        new SpinShooter(shooter, speed)
    );
  }
}

