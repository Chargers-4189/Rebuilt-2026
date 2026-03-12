// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.passing;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.NetworkTables.HoodTable;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.util.NetworkTables.ShooterTable;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.commands.intake.IntakeRotate;
import frc.robot.commands.scoring.AlignAngle;
import frc.robot.commands.scoring.LoadFuel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pass extends ParallelCommandGroup {
  
  /** Creates a new ShootNoSwerveAlign. */
  public Pass(Shooter shooter, Hood hood, Indexer indexer, Hopper hopper, Intake intake, SwerveSubsystem swerve) {
    addCommands(
        new SequentialCommandGroup(Commands.waitSeconds(.5), new LoadFuel(indexer, hopper, shooter, swerve, false)),
        new SpinShooter(shooter, ShooterTable.kPassVelocity),
        hood.setHoodAngleCommand(HoodTable.kPassAngle),
        new SequentialCommandGroup(Commands.waitSeconds(IntakeTable.kTauntDelay.get()), new IntakeRotate(intake, IntakeTable.kTauntRotations))
    );
  }
}

