package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FloorConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PassingCalculatorConstants;
import frc.robot.Constants.ShootingCalculatorConstants;
import frc.robot.Constants.SwerveConstants;

public class NetworkTables {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    public static final NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    public static final Field2d field2d = new Field2d();

    private static CommandXboxController primaryController;

    public static DoubleEntry createEntry(NetworkTable table, String name, double defaultValue) {
        DoubleEntry entry = table.getDoubleTopic(name).getEntry(defaultValue);
        entry.set(entry.get());
        return entry;
    }

    public static BooleanEntry createEntry(NetworkTable table, String name, boolean defaultValue) {
        BooleanEntry entry = table.getBooleanTopic(name).getEntry(defaultValue);
        entry.set(entry.get());
        return entry;
    }

    public static StructEntry<Pose2d> createEntry(NetworkTable table, String name, Pose2d defaultValue) {
        StructEntry<Pose2d> entry = table.getStructTopic(name, Pose2d.struct).getEntry(defaultValue);
        entry.set(entry.get());
        return entry;
    }

    public static class SwerveTable {
        private static final NetworkTable swerveTable = networkInstance.getTable("swerveTable");

        public static final DoublePublisher hubDistance = swerveTable.getDoubleTopic("Hub Distance").publish();
        public static final DoublePublisher hubRotation = swerveTable.getDoubleTopic("Hub Rotation").publish();
        public static final DoublePublisher robotRotation = swerveTable.getDoubleTopic("Robot Rotation").publish();
        public static final DoublePublisher trenchDistance = swerveTable.getDoubleTopic("Trench Distance").publish();

        public static final StringPublisher encoderOffsets = swerveTable.getStringTopic("Encoder Offsets").publish();

        public static final DoubleEntry kAngleP = createEntry(swerveTable, "Angle P (Swerve)", SwerveConstants.kAngleP);
        public static final DoubleEntry kAngleI = createEntry(swerveTable, "Angle I (Swerve)", SwerveConstants.kAngleI);
        public static final DoubleEntry kAngleD = createEntry(swerveTable, "Angle D (Swerve)", SwerveConstants.kAngleD);
        public static final DoubleEntry kAngleS = createEntry(swerveTable, "Angle S (Swerve)", SwerveConstants.kAngleS);
        public static final DoubleEntry kAngleMaxPower = createEntry(swerveTable, "Angle Max Power (Swerve)", SwerveConstants.kAngleMaxPower);
        public static final DoubleEntry kAngleTolerance = createEntry(swerveTable, "Angle Tolerance (Swerve)", SwerveConstants.kAngleTolerance);

        public static final DoubleEntry kPositionP = createEntry(swerveTable, "Position P (Swerve)", SwerveConstants.kPositionP);
        public static final DoubleEntry kPositionI = createEntry(swerveTable, "Position I (Swerve)", SwerveConstants.kPositionI);
        public static final DoubleEntry kPositionD = createEntry(swerveTable, "Position D (Swerve)", SwerveConstants.kPositionD);
        public static final DoubleEntry kPositionS = createEntry(swerveTable, "Position S (Swerve)", SwerveConstants.kPositionS);
        public static final DoubleEntry kPositionMaxPower = createEntry(swerveTable, "Position Max Power (Swerve)", SwerveConstants.kPositionMaxPower);
        public static final DoubleEntry kPositionTolerance = createEntry(swerveTable, "Position Tolerance (Swerve)", SwerveConstants.kPositionTolerance);

        public static final DoubleEntry kDriveExponent = createEntry(swerveTable, "Manual Drive Exponent", SwerveConstants.kDriveExponent);
        public static final DoubleEntry kRotationalExponent = createEntry(swerveTable, "Manual Rotational Exponent", SwerveConstants.kRotationalExponent);

        public static final StructEntry<Pose2d> goalPose = createEntry(swerveTable, "Goal Pose", new Pose2d());

        public static final StructEntry<Pose2d> robotPose = createEntry(swerveTable, "Robot Position", new Pose2d());

        public static final DoubleEntry kExtraRotationSeconds = createEntry(swerveTable, "Extra Rotation Seconds", SwerveConstants.kExtraRotationSeconds);
    }

    public static class IntakeTable {
        private static final NetworkTable intakeTable = networkInstance.getTable("intakeTable");

        public static final DoublePublisher rawEncoder = intakeTable.getDoubleTopic("Intake Raw Encoder").publish();
        public static final DoublePublisher offsetEncoder = intakeTable.getDoubleTopic("Offset Encoder").publish();
        public static final DoublePublisher extensionGoal = intakeTable.getDoubleTopic("Intake Extension Goal").publish();

        public static final DoubleEntry kWheelPower = createEntry(intakeTable, "Intake Wheel Power", IntakeConstants.kWheelPower);
        public static final DoubleEntry kLowWheelPower = createEntry(intakeTable, "Intake Low Wheel Power", IntakeConstants.kLowWheelPower);
        public static final DoubleEntry kManualExtensionPower = createEntry(intakeTable, "Intake Manual Extension Power", IntakeConstants.kManualExtensionPower);
    
        public static final DoubleEntry kAutoOutPower = createEntry(intakeTable, "Intake Auto Out Power", IntakeConstants.kAutoOutPower);
        public static final DoubleEntry kAutoInPower = createEntry(intakeTable, "Intake Auto In Power", IntakeConstants.kAutoInPower);
        
        public static final DoubleEntry kEncoderOffset = createEntry(intakeTable, "Intake Encoder Offset", IntakeConstants.kEncoderOffset);

        public static final DoubleEntry kP = createEntry(intakeTable, "P (Intake)", IntakeConstants.kP);
        public static final DoubleEntry kI = createEntry(intakeTable, "I (Intake)", IntakeConstants.kI);
        public static final DoubleEntry kD = createEntry(intakeTable, "D (Intake)", IntakeConstants.kD);
        public static final DoubleEntry kS = createEntry(intakeTable, "S (Intake)", IntakeConstants.kS);

        public static final DoubleEntry kMaxVelocity = createEntry(intakeTable, "Max Velocity (Intake)", IntakeConstants.kMaxVelocity);
        public static final DoubleEntry kMaxAcceleration = createEntry(intakeTable, "Max Accel. (Intake)", IntakeConstants.kMaxAcceleration);

        public static final BooleanEntry kReverseEncoder = createEntry(intakeTable, "Reverse Intake Encoder", IntakeConstants.reverseEncoder);

        public static final DoubleEntry kTauntRotations = createEntry(intakeTable, "Taunt Rotations", IntakeConstants.kTauntRotations);
        public static final DoubleEntry kTauntFrequency = createEntry(intakeTable, "Taunt Frequency", IntakeConstants.kTauntFrequency);
        public static final DoubleEntry kTauntMagnitude = createEntry(intakeTable, "Taunt Magnitude", IntakeConstants.kTauntMagnitude);

        public static final DoubleEntry kTolerance = createEntry(intakeTable, "Intake Tolerance", IntakeConstants.kTolerance);
        public static final DoubleEntry kOuterExtensionLimit = createEntry(intakeTable, "Intake Outer Limit", IntakeConstants.kOuterExtensionLimit);
        public static final DoubleEntry kInnerExtensionLimit = createEntry(intakeTable, "Intake Inner Limit", IntakeConstants.kInnerExtensionLimit);

        public static final DoubleEntry kTauntDelay = createEntry(intakeTable, "Taunt Delay", IntakeConstants.kTauntDelay);

        public static final DoubleEntry kPushDownPower = createEntry(intakeTable, "Push Down Power", IntakeConstants.kPushDownPower);
    }

    public static class HopperTable {
        private static final NetworkTable hopperTable = networkInstance.getTable("hopperTable");

        public static final DoubleEntry kPower = createEntry(hopperTable, "Hopper Power", FloorConstants.kPower);
        public static final DoubleEntry kReversePower = createEntry(hopperTable, "Hopper Power", FloorConstants.kReversePower);
    }
    
    
    public static class IndexerTable {
        private static final NetworkTable indexerTable = networkInstance.getTable("indexerTable");

        public static final DoubleEntry kPower = createEntry(indexerTable, "Indexer Power", IndexerConstants.kPower);
        public static final DoubleEntry kReversePower = createEntry(indexerTable, "Indexer Reverse Power", IndexerConstants.kReversePower);
    }

    public static class HoodTable {
        private static final NetworkTable hoodTable = networkInstance.getTable("hoodTable");

        public static final DoublePublisher hoodEncoder = hoodTable.getDoubleTopic("Hood Encoder").publish();
        public static final DoublePublisher hoodGoal = hoodTable.getDoubleTopic("Hood Goal").publish();

        public static final DoubleEntry kManualPower = createEntry(hoodTable, "Hood Manual Power", HoodConstants.kManualPower);
        public static final DoubleEntry kAutoPower = createEntry(hoodTable, "Hood Auto Power", HoodConstants.kAutoPower);
        public static final DoubleEntry kDefaultAngle = createEntry(hoodTable, "Hood Default Angle", HoodConstants.kDefaultAngle);
        
        public static final DoubleEntry kP = createEntry(hoodTable, "P (Hood)", HoodConstants.kP);
        public static final DoubleEntry kI = createEntry(hoodTable, "I (Hood)", HoodConstants.kI);
        public static final DoubleEntry kD = createEntry(hoodTable, "D (Hood)", HoodConstants.kD);
    }

    
    public static class FlywheelTable {
        private static final NetworkTable shooterTable = networkInstance.getTable("shooterTable");

        public static final DoublePublisher velocity = shooterTable.getDoubleTopic("Shooter Velocity").publish();
        public static final DoublePublisher velocityGoal = shooterTable.getDoubleTopic("Shooter Velocity Goal").publish();
        public static final DoublePublisher currentPower = shooterTable.getDoubleTopic("Current Power").publish();

        public static final DoubleEntry kSuperSpinPower = createEntry(shooterTable, "Super Spin Power", FlywheelConstants.kSuperSpinPower);
        public static final DoubleEntry kFixedShootDistance = createEntry(shooterTable, "Fixed Shoot Distance", FlywheelConstants.kFixedShootDistance);
        public static final DoubleEntry kPassVelocity = createEntry(shooterTable, "Pass Velocity", FlywheelConstants.kPassVelocity);

        public static final DoubleEntry kTolerance = createEntry(shooterTable, "Shooter Tolerance", FlywheelConstants.kTolerance);

        //slot 0 configs
        public static final DoubleEntry kS = createEntry(shooterTable, "S (Shooter)", FlywheelConstants.kS); // Add 0.25 V output to overcome static friction
        public static final DoubleEntry kV = createEntry(shooterTable, "V (Shooter)", FlywheelConstants.kV); // A velocity target of 1 rps results in 0.12 V output
        public static final DoubleEntry kA = createEntry(shooterTable, "A (Shooter)", FlywheelConstants.kA); // An acceleration of 1 rps/s requires 0.01 V output
        public static final DoubleEntry kP = createEntry(shooterTable, "P (Shooter)", FlywheelConstants.kP);
        public static final DoubleEntry kI = createEntry(shooterTable, "I (Shooter)", FlywheelConstants.kI);
        public static final DoubleEntry kD = createEntry(shooterTable, "D (Shooter)", FlywheelConstants.kD);

        // set Motion Magic settings
        public static final DoubleEntry kMotionMagicCruiseVelocity = createEntry(shooterTable, "MM Velocity (Shooter)", FlywheelConstants.kMotionMagicCruiseVelocity); // Target cruise velocity of 80 rps
        public static final DoubleEntry kMotionMagicAcceleration = createEntry(shooterTable, "MM Acceleration (Shooter)", FlywheelConstants.kMotionMagicAcceleration); // Target acceleration of 160 rps/s (0.5 seconds)
        public static final DoubleEntry kMotionMagicJerk = createEntry(shooterTable, "MM Jerk (Shooter)", FlywheelConstants.kMotionMagicJerk); // Target jerk of 1600 rps/s/s (0.1 seconds)

        public static final DoubleEntry kMaxPowerCutoff = createEntry(shooterTable, "Max Power Range", FlywheelConstants.KMaxPowerCutoff);
    }

    public static class ShootingCalculatorTable {
        private static final NetworkTable shootingCalcTable = networkInstance.getTable("shootingCalcTable");

        public static final DoubleEntry kAngleIntercept = createEntry(shootingCalcTable, "Intercept (Score Angle)", ShootingCalculatorConstants.kAngleIntercept);
        public static final DoubleEntry kAngleSlope = createEntry(shootingCalcTable, "Slope (Score Angle)", ShootingCalculatorConstants.kAngleSlope);
        public static final DoubleEntry kVelocitySlope = createEntry(shootingCalcTable, "Slope (Score Velocity)", ShootingCalculatorConstants.kVelocitySlope);
        public static final DoubleEntry kVelocityIntercept = createEntry(shootingCalcTable, "Intercept (Score Velocity)", ShootingCalculatorConstants.kVelocityIntercept);
        public static final DoubleEntry kVelocitySquared = createEntry(shootingCalcTable, "Squared (Score Velocity)", ShootingCalculatorConstants.kVelocitySquared);
    }

    public static class PassingCalculatorTable {
        private static final NetworkTable passingCalcTable = networkInstance.getTable("passingCalcTable");

        public static final DoubleEntry kAngleIntercept = createEntry(passingCalcTable, "Intercept (Pass Angle))", PassingCalculatorConstants.kAngleIntercept);
        public static final DoubleEntry kAngleSlope = createEntry(passingCalcTable, "Slope (Pass Angle))", PassingCalculatorConstants.kAngleSlope);
        public static final DoubleEntry kVelocitySlope = createEntry(passingCalcTable, "Slope (Pass Velocity)", PassingCalculatorConstants.kVelocitySlope);
        public static final DoubleEntry kVelocityIntercept = createEntry(passingCalcTable, "Intercept (Pass Velocity)", PassingCalculatorConstants.kVelocityIntercept);
        public static final DoubleEntry kMinVelocity = createEntry(passingCalcTable, "Min (Pass Velocity)", PassingCalculatorConstants.kMinVelocity);
        public static final DoubleEntry kMaxVelocity = createEntry(passingCalcTable, "Max (Pass Velocity)", PassingCalculatorConstants.kMaxVelocity);
        public static final DoubleEntry kMinHoodAngle = createEntry(passingCalcTable, "Min (Pass Angle)", PassingCalculatorConstants.kMinHoodAngle);
        public static final DoubleEntry kMaxHoodAngle = createEntry(passingCalcTable, "Max (Pass Angle)", PassingCalculatorConstants.kMaxHoodAngle);
    }

    public static class AutoTable {
        private static final NetworkTable autoTable = networkInstance.getTable("autoTable");

        public static final BooleanEntry kRightSide = createEntry(autoTable, "Right Side", AutoConstants.kRightSide);
        
        public static final DoubleEntry kPreSpinDuration = createEntry(autoTable, "Pre Spin Duration", AutoConstants.kPreSpinDuration);
        public static final DoubleEntry kPreSpinVelocity = createEntry(autoTable, "Pre Spin Velocity", AutoConstants.kPreSpinVelocity);
        public static final DoubleEntry kShooterTimeout = createEntry(autoTable, "Shooter Timeout", AutoConstants.kShooterTimeout);
    }
    
    
    public static void addController(CommandXboxController newPrimaryController) {
        primaryController = newPrimaryController;
    }

    public static void periodic() {
        field2d.setRobotPose(SwerveTable.robotPose.get());
        field2d.getObject("Goal Pose").setPose(SwerveTable.goalPose.get());
        SmartDashboard.putData(field2d);
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(primaryController.getHID());
        SmartDashboard.putBoolean("Auto Winner", DriverStation.getGameSpecificMessage() == "R");
        SmartDashboard.putBoolean("Hub Active", HelpfulFunctions.isHubActive());
    }
}
