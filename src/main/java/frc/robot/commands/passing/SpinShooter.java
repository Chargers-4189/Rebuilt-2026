// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.passing;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinShooter extends Command {

  private Shooter shooter;
  private DoubleSupplier velocity;
  private boolean stopMovingOnEnd;

  /** Creates a new Shoot. */
  public SpinShooter(Shooter shooter, DoubleSupplier velocity, boolean stopMovingOnEnd) {
    this.shooter = shooter;
    this.velocity = velocity;
    this.stopMovingOnEnd = stopMovingOnEnd;
    addRequirements(shooter);
  }

  public SpinShooter(Shooter shooter, DoubleSupplier power) {
    this(shooter, power, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      shooter.setVelocitySimple(velocity.getAsDouble());
      System.out.println(velocity.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopMovingOnEnd) {
      shooter.setVelocitySimple(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
