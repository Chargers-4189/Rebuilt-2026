// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.ScoreAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.NetworkTables.SwerveTable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExampleAutoScore extends SequentialCommandGroup {
  /** Creates a new ExampleAutoScore. */
  public ExampleAutoScore(Shooter shooter, Hood hood, Indexer indexer, SwerveSubsystem swerve, Vision vision, Hopper hopper) {
    double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    SwerveRequest.FieldCentricFacingAngle driveWithAngle = new SwerveRequest.FieldCentricFacingAngle()
              .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
              .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    Command drive = swerve.applyRequest(()->driveWithAngle.withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withTargetDirection(vision.getRotationFromHub())
                    .withHeadingPID(SwerveTable.kP.get(), SwerveTable.kI.get(), SwerveTable.kD.get())
                    .withMaxAbsRotationalRate(MaxAngularRate * SwerveTable.kMaxPower.get())
                    .withTargetRateFeedforward(swerve.calculateFeedForward(vision.getRotationFromHub()))
            );
    Command path;
    try {
        path = AutoBuilder.followPath(PathPlannerPath.fromPathFile("test"));
    } catch(Exception e){
        path = Commands.none();
    }
    addCommands(
        path,
        new ParallelCommandGroup(drive.withTimeout(5),new ScoreAuto(shooter, hood, indexer, swerve, vision, hopper).withTimeout(5))
    );
  }
}
