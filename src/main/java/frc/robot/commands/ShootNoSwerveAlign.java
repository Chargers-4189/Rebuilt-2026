// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.util.NetworkTables.HoodTable;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Hood;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoSwerveAlign extends ParallelCommandGroup {
  /** Creates a new test. */
  private Hood hood;
  private Indexer indexer;
  private Shooter shooter;

  public ShootNoSwerveAlign(Shooter shooter, Hood hood, Indexer indexer, double speed, double angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.hood = hood;
    this.indexer = indexer;
    this.shooter = shooter;
    addCommands(
        new SequentialCommandGroup(Commands.waitSeconds(3), new MoveIndexer(indexer, shooter)),
        //hood.setHoodAngleCommand(() -> HoodTable.kTestAngle.get()),
        new Shoot(shooter, speed)
    );
  }
}

