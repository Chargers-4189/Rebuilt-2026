// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.NetworkTables.SwerveTable;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignPosition extends Command {

  private SwerveSubsystem swerve;

  private double xPower;
  private double yPower;
  private double anglePower;

  private Pose2d goalPose;
  private Pose2d currentPose;

  private PIDController xPid = new PIDController(0, 0, 0);
  private PIDController yPid = new PIDController(0, 0, 0);
  private PIDController anglePid = new PIDController(0, 0, 0);


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  /** Creates a new AutoAlignPose. */
  public AlignPosition(SwerveSubsystem swerve, Pose2d goalPose) {
    this.swerve = swerve;
    this.goalPose = goalPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePid.enableContinuousInput(-1, 1);
    xPid.setPID(SwerveTable.kPositionP.get(), SwerveTable.kPositionI.get(), SwerveTable.kPositionD.get());
    yPid.setPID(SwerveTable.kPositionP.get(), SwerveTable.kPositionI.get(), SwerveTable.kPositionD.get());
    anglePid.setPID(SwerveTable.kAngleP.get(), SwerveTable.kAngleI.get(), SwerveTable.kAngleD.get());

    xPid.setTolerance(SwerveTable.kPositionTolerance.get());
    yPid.setTolerance(SwerveTable.kPositionTolerance.get());
    anglePid.setTolerance(SwerveTable.kAngleTolerance.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = swerve.getPose();

    xPower = MathUtil.clamp(-xPid.calculate(currentPose.getX(), goalPose.getX()), -SwerveTable.kPositionMaxPower.get(), SwerveTable.kPositionMaxPower.get());
    yPower = MathUtil.clamp(-yPid.calculate(currentPose.getY(), goalPose.getY()), -SwerveTable.kPositionMaxPower.get(), SwerveTable.kPositionMaxPower.get());
    anglePower = MathUtil.clamp(
      anglePid.calculate(currentPose.getRotation().getRotations(), goalPose.getRotation().getRotations()),
      -SwerveTable.kAngleMaxPower.get(),
      SwerveTable.kAngleMaxPower.get()
    );

    swerve.setControl(
      drive.withVelocityX(xPower * swerve.MaxSpeed)
           .withVelocityY(yPower * swerve.MaxSpeed)
           .withRotationalRate(anglePower * swerve.MaxAngularRate)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setControl(
      drive.withVelocityX(0)
           .withVelocityY(0)
           .withRotationalRate(0)
    );
  }

  // Returns true when the command should end.P
  @Override
  public boolean isFinished() {
    return anglePid.atSetpoint() && xPid.atSetpoint() && yPid.atSetpoint();
  }
}
