// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.commands.intake.IntakeRotate;
import frc.robot.commands.intake.RunIntakeWheels;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score extends ParallelCommandGroup {

  /** Creates a new Score. */
  public Score(Shooter shooter, Hood hood, Indexer indexer, SwerveSubsystem swerve, Vision vision, Hopper hopper, DoubleSupplier driveX, DoubleSupplier driveY) {
    addCommands(
        new SequentialCommandGroup(Commands.waitSeconds(.5), new LoadFuel(indexer, hopper, shooter, swerve, true)),
        new AlignHoodAndFlywheel(hood, shooter, vision),
        new AlignAngle(swerve, driveX, driveY, () -> vision.getRotationFromHub(), false)
    );
  }

  /** Creates a new Score. */
  public Score(Shooter shooter, Hood hood, Indexer indexer, SwerveSubsystem swerve, Vision vision, Hopper hopper, CommandXboxController primaryController) {
    this(shooter, hood, indexer, swerve, vision, hopper, () -> -primaryController.getLeftY(), () -> -primaryController.getLeftX());
  }

  /** Creates a new Score. */
  public Score(Shooter shooter, Hood hood, Indexer indexer, SwerveSubsystem swerve, Vision vision, Hopper hopper) {
    this(shooter, hood, indexer, swerve, vision, hopper, () -> 0, () -> 0);
  }
}