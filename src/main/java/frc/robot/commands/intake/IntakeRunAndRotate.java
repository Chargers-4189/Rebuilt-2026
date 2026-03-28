// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.util.NetworkTables.IntakeTable;
import frc.robot.subsystems.IntakeExtender;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRunAndRotate extends ParallelCommandGroup {
  /** Creates a new IntakeRunAndRotate2. */
  public IntakeRunAndRotate(IntakeWheels intakeWheels, IntakeExtender intakeExtender, DoubleSupplier power) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        new IntakeRotate(intakeExtender, true),
        intakeExtender.setPowerCommand(IntakeTable.kPushDownPower)
      ),
      intakeWheels.runWheelsCommand(power)
    );
  }
}
