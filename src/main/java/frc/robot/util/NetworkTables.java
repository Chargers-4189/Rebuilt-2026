package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;

public class NetworkTables {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    public static final NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    public static final Field2d field2d = new Field2d();

    private static CommandXboxController primaryController;

    public static class SwerveTable {
        private static final NetworkTable swerveTable = networkInstance.getTable("swerveTable");

        public static final StructEntry<Pose2d> robotPose = swerveTable.getStructTopic("Robot Position", Pose2d.struct).getEntry(new Pose2d());

        public static final DoublePublisher hubDistance = swerveTable.getDoubleTopic("Hub Distance").publish();
        public static final DoublePublisher hubRotation = swerveTable.getDoubleTopic("Hub Rotation").publish();
        public static final DoublePublisher robotRotation = swerveTable.getDoubleTopic("Robot Rotation").publish();

        public static final DoubleEntry kAngleP = swerveTable.getDoubleTopic("Angle P (Swerve)").getEntry(Constants.SwerveConstants.kAngleP);
        public static final DoubleEntry kAngleI = swerveTable.getDoubleTopic("Angle I (Swerve)").getEntry(Constants.SwerveConstants.kAngleI);
        public static final DoubleEntry kAngleD = swerveTable.getDoubleTopic("Angle D (Swerve)").getEntry(Constants.SwerveConstants.kAngleD);
        public static final DoubleEntry kAngleS = swerveTable.getDoubleTopic("Angle S (Swerve)").getEntry(Constants.SwerveConstants.kAngleS);
        public static final DoubleEntry kAngleMaxPower = swerveTable.getDoubleTopic("Angle Max Power (Swerve)").getEntry(Constants.SwerveConstants.kAngleMaxPower);
        public static final DoubleEntry kAngleTolerance = swerveTable.getDoubleTopic("Angle Tolerance (Swerve)").getEntry(Constants.SwerveConstants.kAngleTolerance);

        public static final DoubleEntry kPositionP = swerveTable.getDoubleTopic("Position P (Swerve)").getEntry(Constants.SwerveConstants.kPositionP);
        public static final DoubleEntry kPositionI = swerveTable.getDoubleTopic("Position I (Swerve)").getEntry(Constants.SwerveConstants.kPositionI);
        public static final DoubleEntry kPositionD = swerveTable.getDoubleTopic("Position D (Swerve)").getEntry(Constants.SwerveConstants.kPositionD);
        public static final DoubleEntry kPositionS = swerveTable.getDoubleTopic("Position S (Swerve)").getEntry(Constants.SwerveConstants.kPositionS);
        public static final DoubleEntry kPositionMaxPower = swerveTable.getDoubleTopic("Position Max Power (Swerve)").getEntry(Constants.SwerveConstants.kPositionMaxPower);
        public static final DoubleEntry kPositionTolerance = swerveTable.getDoubleTopic("Position Tolerance (Swerve)").getEntry(Constants.SwerveConstants.kPositionTolerance);

        public static final DoubleEntry kDriveExponent = swerveTable.getDoubleTopic("Position Drive Exponent").getEntry(Constants.SwerveConstants.kDriveExponent);
        public static final DoubleEntry kRotationalExponent = swerveTable.getDoubleTopic("Position Rotational Exponent").getEntry(Constants.SwerveConstants.kRotationalExponent);

        public static final StringPublisher encoderOffsets = swerveTable.getStringTopic("Encoder Offsets").publish();

        public static void init() {
            kAngleP.set(kAngleP.get());
            kAngleI.set(kAngleI.get());
            kAngleD.set(kAngleD.get());
            kAngleS.set(kAngleS.get());
            kAngleMaxPower.set(kAngleMaxPower.get());
            kAngleTolerance.set(kAngleTolerance.get());

            kPositionP.set(kPositionP.get());
            kPositionI.set(kPositionI.get());
            kPositionD.set(kPositionD.get());
            kPositionS.set(kPositionS.get());
            kPositionMaxPower.set(kPositionMaxPower.get());
            kPositionTolerance.set(kPositionTolerance.get());

            kDriveExponent.set(kDriveExponent.get());
            kRotationalExponent.set(kRotationalExponent.get());
        }
    }

    public static class IntakeTable {
        private static final NetworkTable intakeTable = networkInstance.getTable("intakeTable");

        public static final DoublePublisher rawEncoder = intakeTable.getDoubleTopic("Intake Raw Encoder").publish();
        public static final DoublePublisher offsetEncoder = intakeTable.getDoubleTopic("Offset Encoder").publish();
        public static final DoublePublisher extensionGoal = intakeTable.getDoubleTopic("Intake Extension Goal").publish();

        public static final DoubleEntry kWheelPower = intakeTable.getDoubleTopic("Intake Wheel Power").getEntry(Constants.IntakeConstants.kWheelPower);
        public static final DoubleEntry kManualExtensionPower = intakeTable.getDoubleTopic("Intake Manual Extension Power").getEntry(Constants.IntakeConstants.kManualExtensionPower);
    
        public static final DoubleEntry kDefaultAngle = intakeTable.getDoubleTopic("Intake Default Angle").getEntry(Constants.IntakeConstants.kDefaultAngle);

        public static final DoubleEntry kAutoOutPower = intakeTable.getDoubleTopic("Intake Auto Out Power").getEntry(Constants.IntakeConstants.kAutoOutPower);
        public static final DoubleEntry kAutoInPower = intakeTable.getDoubleTopic("Intake Auto In Power").getEntry(Constants.IntakeConstants.kAutoInPower);

        public static final DoubleEntry kP = intakeTable.getDoubleTopic("P (Intake)").getEntry(Constants.IntakeConstants.kP);
        public static final DoubleEntry kI = intakeTable.getDoubleTopic("I (Intake)").getEntry(Constants.IntakeConstants.kI);
        public static final DoubleEntry kD = intakeTable.getDoubleTopic("D (Intake)").getEntry(Constants.IntakeConstants.kD);
        public static final DoubleEntry kS = intakeTable.getDoubleTopic("S (Intake)").getEntry(Constants.IntakeConstants.kS);

        public static final DoubleEntry kTauntRotations = intakeTable.getDoubleTopic("Taunt Rotations").getEntry(Constants.IntakeConstants.kTauntRotations);

        public static final DoubleEntry kTolerance = intakeTable.getDoubleTopic("Intake Tolerance").getEntry(Constants.IntakeConstants.kTolerance);
        public static final DoubleEntry kOuterExtensionLimit = intakeTable.getDoubleTopic("Intake Outer Limit").getEntry(Constants.IntakeConstants.kOuterExtensionLimit);
        public static final DoubleEntry kInnerExtensionLimit = intakeTable.getDoubleTopic("Intake Inner Limit").getEntry(Constants.IntakeConstants.kInnerExtensionLimit);

        public static final DoubleEntry kTauntDelay = intakeTable.getDoubleTopic("Taunt Delay").getEntry(Constants.IntakeConstants.kTauntDelay);

        public static void init() {
            kWheelPower.set(kWheelPower.get());
            kManualExtensionPower.set(kManualExtensionPower.get());

            kDefaultAngle.set(kDefaultAngle.get());

            kAutoInPower.set(kAutoInPower.get());
            kAutoOutPower.set(kAutoOutPower.get());

            kP.set(kP.get());
            kI.set(kI.get());
            kD.set(kD.get());
            kS.set(kS.get());

            kTauntRotations.set(kTauntRotations.get());

            kTolerance.set(kTolerance.get());
            kOuterExtensionLimit.set(kOuterExtensionLimit.get());
            kInnerExtensionLimit.set(kInnerExtensionLimit.get());

            kTauntDelay.set(kTauntDelay.get());
        }
    }

    public static class HopperTable {
        private static final NetworkTable hopperTable = networkInstance.getTable("hopperTable");

        public static final DoubleEntry kPower = hopperTable.getDoubleTopic("Hopper Power").getEntry(Constants.HopperConstants.kPower);
        public static final DoubleEntry kReversePower = hopperTable.getDoubleTopic("Hopper Power").getEntry(Constants.HopperConstants.kReversePower);

        public static final void init() {
            kPower.set(kPower.get());
            kReversePower.set(kReversePower.get());
        }
    }
    
    
    public static class IndexerTable {
        private static final NetworkTable indexerTable = networkInstance.getTable("indexerTable");

        public static final DoubleEntry kPower = indexerTable.getDoubleTopic("Indexer Power").getEntry(Constants.IndexerConstants.kPower);
        public static final DoubleEntry kReversePower = indexerTable.getDoubleTopic("Indexer Reverse Power").getEntry(Constants.IndexerConstants.kReversePower);

        public static final void init() {
            kPower.set(kPower.get());
            kReversePower.set(kReversePower.get());
        }
    }

    public static class HoodTable {
        private static final NetworkTable hoodTable = networkInstance.getTable("hoodTable");

        public static final DoublePublisher hoodEncoder = hoodTable.getDoubleTopic("Hood Encoder").publish();
        public static final DoublePublisher hoodGoal = hoodTable.getDoubleTopic("Hood Goal").publish();

        public static final DoubleEntry kManualPower = hoodTable.getDoubleTopic("Hood Manual Power").getEntry(Constants.HoodConstants.kManualPower);
        public static final DoubleEntry kAutoPower = hoodTable.getDoubleTopic("Hood Auto Power").getEntry(Constants.HoodConstants.kAutoPower);
        public static final DoubleEntry kDefaultAngle = hoodTable.getDoubleTopic("Hood Default Angle").getEntry(Constants.HoodConstants.kDefaultAngle);
        
        public static final DoubleEntry kPassAngle = hoodTable.getDoubleTopic("Pass Angle").getEntry(Constants.HoodConstants.kPassAngle);

        public static final DoubleEntry kP = hoodTable.getDoubleTopic("P (Hood)").getEntry(Constants.HoodConstants.kP);
        public static final DoubleEntry kI = hoodTable.getDoubleTopic("I (Hood)").getEntry(Constants.HoodConstants.kI);
        public static final DoubleEntry kD = hoodTable.getDoubleTopic("D (Hood)").getEntry(Constants.HoodConstants.kD);


        public static final void init() {
            kManualPower.set(kManualPower.get());
            kAutoPower.set(kAutoPower.get());
            kDefaultAngle.set(kDefaultAngle.get());

            kPassAngle.set(kPassAngle.get());

            kP.set(kP.get());
            kI.set(kI.get());
            kD.set(kD.get());
        }
    }

    
    public static class ShooterTable {
        private static final NetworkTable shooterTable = networkInstance.getTable("shooterTable");

        public static final DoublePublisher velocity = shooterTable.getDoubleTopic("Shooter Velocity").publish();
        public static final DoublePublisher velocityGoal = shooterTable.getDoubleTopic("Shooter Velocity Goal").publish();
        public static final DoublePublisher currentPower = shooterTable.getDoubleTopic("Current Power").publish();

        public static final DoubleEntry kTestPower = shooterTable.getDoubleTopic("Shooter Test Power").getEntry(Constants.ShooterConstants.kFixedPower);
        public static final DoubleEntry kFixedShootDistance = shooterTable.getDoubleTopic("Fixed Shoot Distance").getEntry(Constants.ShooterConstants.kFixedShootDistance);
        public static final DoubleEntry kPassVelocity = shooterTable.getDoubleTopic("Pass Velocity").getEntry(Constants.ShooterConstants.kPassVelocity);

        public static final DoubleEntry kTolerance = shooterTable.getDoubleTopic("Shooter Tolerance").getEntry(Constants.ShooterConstants.kTolerance);

        //slot 0 configs
        public static final DoubleEntry kS = shooterTable.getDoubleTopic("S (Shooter)").getEntry(Constants.ShooterConstants.kS);; // Add 0.25 V output to overcome static friction
        public static final DoubleEntry kV = shooterTable.getDoubleTopic("V (Shooter)").getEntry(Constants.ShooterConstants.kV);; // A velocity target of 1 rps results in 0.12 V output
        public static final DoubleEntry kA = shooterTable.getDoubleTopic("A (Shooter)").getEntry(Constants.ShooterConstants.kA);; // An acceleration of 1 rps/s requires 0.01 V output
        public static final DoubleEntry kP = shooterTable.getDoubleTopic("P (Shooter)").getEntry(Constants.ShooterConstants.kP);
        public static final DoubleEntry kI = shooterTable.getDoubleTopic("I (Shooter)").getEntry(Constants.ShooterConstants.kI);
        public static final DoubleEntry kD = shooterTable.getDoubleTopic("D (Shooter)").getEntry(Constants.ShooterConstants.kD);

        // set Motion Magic settings
        public static final DoubleEntry kMotionMagicCruiseVelocity = shooterTable.getDoubleTopic("MM Velocity (Shooter)").getEntry(Constants.ShooterConstants.kMotionMagicCruiseVelocity);; // Target cruise velocity of 80 rps
        public static final DoubleEntry kMotionMagicAcceleration = shooterTable.getDoubleTopic("MM Acceleration (Shooter)").getEntry(Constants.ShooterConstants.kMotionMagicAcceleration);; // Target acceleration of 160 rps/s (0.5 seconds)
        public static final DoubleEntry kMotionMagicJerk = shooterTable.getDoubleTopic("MM Jerk (Shooter)").getEntry(Constants.ShooterConstants.kMotionMagicJerk);; // Target jerk of 1600 rps/s/s (0.1 seconds)

        public static final void init() {
            kTestPower.set(kTestPower.get());
            kFixedShootDistance.set(kFixedShootDistance.get());
            kPassVelocity.set(kPassVelocity.get());

            kTolerance.set(kTolerance.get());

            kS.set(kS.get());
            kV.set(kV.get());
            kA.set(kA.get());
            kP.set(kP.get());
            kI.set(kI.get());
            kD.set(kD.get());

            kMotionMagicCruiseVelocity.set(kMotionMagicCruiseVelocity.get());
            kMotionMagicAcceleration.set(kMotionMagicAcceleration.get());
            kMotionMagicJerk.set(kMotionMagicJerk.get());
        }
    }

    public static class ShootingCalculatorTable {
        private static final NetworkTable shootingCalcTable = networkInstance.getTable("shootingCalcTable");

        public static final DoubleEntry kHoodIntercept = shootingCalcTable.getDoubleTopic("Intercept (Hood Calc)").getEntry(Constants.ShootingCalculatorConstants.kHoodIntercept);
        public static final DoubleEntry kHoodSlope = shootingCalcTable.getDoubleTopic("Slope (Hood Calc)").getEntry(Constants.ShootingCalculatorConstants.kHoodSlope);
        public static final DoubleEntry kVelocitySlope = shootingCalcTable.getDoubleTopic("Slope (Velocity Calc)").getEntry(Constants.ShootingCalculatorConstants.kVelocitySlope);
        public static final DoubleEntry kVelocityIntercept = shootingCalcTable.getDoubleTopic("Intercept (Velocity Calc)").getEntry(Constants.ShootingCalculatorConstants.kVelocityIntercept);
        public static final DoubleEntry kVelocitySquared = shootingCalcTable.getDoubleTopic("Squared (Velocity Calc)").getEntry(Constants.ShootingCalculatorConstants.kVelocitySquared);

        public static final void init() {
            kHoodIntercept.set(kHoodIntercept.get());
            kHoodSlope.set(kHoodSlope.get());
            kVelocitySlope.set(kVelocitySlope.get());
            kVelocityIntercept.set(kVelocityIntercept.get());
            kVelocitySquared.set(kVelocitySquared.get());
        }
    }

    public static class AutoTable {
        private static final NetworkTable autoTable = networkInstance.getTable("autoTable");


        public static final BooleanEntry kRightSide = autoTable.getBooleanTopic("Intercept (Hood Calc)").getEntry(Constants.AutoConstants.kRightSide);

        public static final void init() {
            kRightSide.set(kRightSide.get());
        }
    }
    
    
    public static void initialize(CommandXboxController primaryController) {
        NetworkTables.primaryController = primaryController;
        SwerveTable.init();
        IntakeTable.init();
        HopperTable.init();
        IndexerTable.init();
        HoodTable.init();
        ShooterTable.init();
        ShootingCalculatorTable.init();
        AutoTable.init();
    }

    public static void periodic() {
        field2d.setRobotPose(SwerveTable.robotPose.get());
        SmartDashboard.putData(field2d);
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(primaryController.getHID());
        SmartDashboard.putBoolean("Auto Winner", DriverStation.getGameSpecificMessage() == "R");
        SmartDashboard.putBoolean("Hub Active", HelpfulFunctions.isHubActive());
    }
}
