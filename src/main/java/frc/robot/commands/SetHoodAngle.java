// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHoodAngle extends Command {

  private Hood hood;
  double angle; //position of hood we currently want
  double currentAngle;
  double hoodTolerance = 0.5;  //This line should eventually be removed and it's applications should be replaced with this number in the constants file
  /** Creates a new ShootFuel. */
  public SetHoodAngle(Hood hood, double Angle) {
    this.hood = hood;
    this.angle = angle;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = hood.getHoodPosition();

    //sets the hood angle to the desired hood angle, if the hood angle is within the tolerance we set, then it shoots the fuel
    if(Math.abs(angle - currentAngle) >= hoodTolerance){
      if(angle <= currentAngle){
        hood.setHoodPower(0.05); // this number (currently 0.05) should probably be iterated then put into constants
      }else if(angle <= currentAngle){
        hood.setHoodPower(-0.05);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setHoodPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(angle - currentAngle) <= hoodTolerance){
      return true;
    }
    return false;
  }
}
