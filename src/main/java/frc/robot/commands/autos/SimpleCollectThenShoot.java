// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeExtender;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleCollectThenShoot extends SequentialCommandGroup {
  /** Creates a new AutoCenterCollectAndShootFullPath. */
  public SimpleCollectThenShoot(Shooter shooter, Hood hood, Indexer indexer, SwerveSubsystem swerve, Vision vision, Hopper hopper,  IntakeWheels intakeWheels, IntakeExtender intakeExtender, ChoreoTraj traj, boolean doublePass) {
    if (doublePass) {
      addCommands(
        new SinglePass(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, traj, true),
        new SinglePass(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, ChoreoTraj.secondPass, false)
      );
    } else {
      addCommands(
        new SinglePass(shooter, hood, indexer, swerve, vision, hopper, intakeWheels, intakeExtender, traj, false)
      );
    }
  }
}
