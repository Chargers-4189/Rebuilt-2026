package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class NetworkTables {
    static NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    static Field2d field2d = new Field2d();

    public static final class SwerveTable {
        static NetworkTable swerveTable = networkInstance.getTable("swerveTable");

        public static StructEntry<Pose2d> robotPose = swerveTable.getStructTopic("Robot Position", Pose2d.struct).getEntry(new Pose2d());

        public static DoublePublisher hubDistance = swerveTable.getDoubleTopic("Hub Distance").publish();
        public static DoublePublisher hubRotation = swerveTable.getDoubleTopic("Hub Rotation").publish();
        public static DoublePublisher robotRotation = swerveTable.getDoubleTopic("Robot Rotation").publish();

        public static DoubleEntry kP = swerveTable.getDoubleTopic("P (Swerve)").getEntry(Constants.SwerveConstants.kP);
        public static DoubleEntry kI = swerveTable.getDoubleTopic("I (Swerve)").getEntry(Constants.SwerveConstants.kI);
        public static DoubleEntry kD = swerveTable.getDoubleTopic("D (Swerve)").getEntry(Constants.SwerveConstants.kD);
        public static DoubleEntry kS = swerveTable.getDoubleTopic("S (Swerve)").getEntry(Constants.SwerveConstants.kS);
        public static DoubleEntry kMaxPower = swerveTable.getDoubleTopic("Max Power (Swerve)").getEntry(Constants.SwerveConstants.kMaxPower);
        public static DoubleEntry kTolerance = swerveTable.getDoubleTopic("Tolerance (Swerve)").getEntry(Constants.SwerveConstants.kTolerance);

        public static DoubleEntry kDriveExponent = swerveTable.getDoubleTopic("Drive Exponent").getEntry(Constants.SwerveConstants.kDriveExponent);
        public static DoubleEntry kRotationalExponent = swerveTable.getDoubleTopic("Rotational Exponent").getEntry(Constants.SwerveConstants.kRotationalExponent);

        public static void init() {
            kP.set(kP.get());
            kI.set(kI.get());
            kD.set(kD.get());
            kS.set(kS.get());
            kMaxPower.set(kMaxPower.get());

            kTolerance.set(kTolerance.get());

            kDriveExponent.set(kDriveExponent.get());
            kRotationalExponent.set(kRotationalExponent.get());
        }
    }

    public static final class IntakeTable {
        static NetworkTable intakeTable = networkInstance.getTable("intakeTable");

        public static DoublePublisher rawEncoder = intakeTable.getDoubleTopic("Intake Raw Encoder").publish();
        public static DoublePublisher extensionGoal = intakeTable.getDoubleTopic("Intake Extension Goal").publish();

        public static DoubleEntry kWheelPower = intakeTable.getDoubleTopic("Intake Wheel Power").getEntry(Constants.IntakeConstants.kWheelPower);
        public static DoubleEntry kManualExtensionPower = intakeTable.getDoubleTopic("Intake Manual Extension Power").getEntry(Constants.IntakeConstants.kManualExtensionPower);
    
        public static DoubleEntry kDefaultAngle = intakeTable.getDoubleTopic("Intake Default Angle").getEntry(Constants.IntakeConstants.kDefaultAngle);

        public static DoubleEntry kAutoOutPower = intakeTable.getDoubleTopic("Intake Auto Out Power").getEntry(Constants.IntakeConstants.kAutoOutPower);
        public static DoubleEntry kAutoInPower = intakeTable.getDoubleTopic("Intake Auto In Power").getEntry(Constants.IntakeConstants.kAutoInPower);

        public static DoubleEntry kP = intakeTable.getDoubleTopic("P (Intake)").getEntry(Constants.IntakeConstants.kP);
        public static DoubleEntry kI = intakeTable.getDoubleTopic("I (Intake)").getEntry(Constants.IntakeConstants.kI);
        public static DoubleEntry kD = intakeTable.getDoubleTopic("D (Intake)").getEntry(Constants.IntakeConstants.kD);

        public static IntegerEntry kTauntAmount = intakeTable.getIntegerTopic("Taunt Number").getEntry(Constants.IntakeConstants.kTauntAmount);
        public static IntegerEntry kTauntFraction = intakeTable.getIntegerTopic("Taunt Fraction").getEntry(Constants.IntakeConstants.kTauntFraction);

        public static DoubleEntry kTolerance = intakeTable.getDoubleTopic("Intake Tolerance").getEntry(Constants.IntakeConstants.kTolerance);
        public static DoubleEntry kOuterExtensionLimit = intakeTable.getDoubleTopic("Intake Outer Limit").getEntry(Constants.IntakeConstants.kOuterExtensionLimit);
        public static DoubleEntry kInnerExtensionLimit = intakeTable.getDoubleTopic("Intake Inner Limit").getEntry(Constants.IntakeConstants.kInnerExtensionLimit);

        public static void init() {
            kWheelPower.set(kWheelPower.get());
            kManualExtensionPower.set(kManualExtensionPower.get());

            kDefaultAngle.set(kDefaultAngle.get());

            kAutoInPower.set(kAutoInPower.get());
            kAutoOutPower.set(kAutoOutPower.get());

            kP.set(kP.get());
            kI.set(kI.get());
            kD.set(kD.get());

            kTauntAmount.set(kTauntAmount.get());
            kTauntFraction.set(kTauntFraction.get());

            kTolerance.set(kTolerance.get());
            kOuterExtensionLimit.set(kOuterExtensionLimit.get());
            kInnerExtensionLimit.set(kInnerExtensionLimit.get());
        }
    }

    public static final class HopperTable {
        static NetworkTable hopperTable = networkInstance.getTable("hopperTable");

        public static DoubleEntry kPower = hopperTable.getDoubleTopic("Hopper Power").getEntry(Constants.HopperConstants.kPower);
        public static DoubleEntry kReversePower = hopperTable.getDoubleTopic("Hopper Power").getEntry(Constants.HopperConstants.kReversePower);

        public static void init() {
            kPower.set(kPower.get());
            kReversePower.set(kReversePower.get());
        }
    }
    
    
    public static final class IndexerTable {
        static NetworkTable indexerTable = networkInstance.getTable("indexerTable");

        public static DoubleEntry kPower = indexerTable.getDoubleTopic("Indexer Power").getEntry(Constants.IndexerConstants.kPower);
        public static DoubleEntry kReversePower = indexerTable.getDoubleTopic("Indexer Reverse Power").getEntry(Constants.IndexerConstants.kReversePower);

        public static void init() {
            kPower.set(kPower.get());
            kReversePower.set(kReversePower.get());
        }
    }

    public static final class HoodTable {
        static NetworkTable hoodTable = networkInstance.getTable("hoodTable");

        public static DoublePublisher hoodEncoder = hoodTable.getDoubleTopic("Hood Encoder").publish();
        public static DoublePublisher hoodGoal = hoodTable.getDoubleTopic("Hood Goal").publish();

        public static DoubleEntry kManualPower = hoodTable.getDoubleTopic("Hood Manual Power").getEntry(Constants.HoodConstants.kManualPower);
        public static DoubleEntry kAutoPower = hoodTable.getDoubleTopic("Hood Auto Power").getEntry(Constants.HoodConstants.kAutoPower);
        public static DoubleEntry kDefaultAngle = hoodTable.getDoubleTopic("Hood Default Angle").getEntry(Constants.HoodConstants.kDefaultAngle);

        public static DoubleEntry kP = hoodTable.getDoubleTopic("P (Hood)").getEntry(Constants.HoodConstants.kP);
        public static DoubleEntry kI = hoodTable.getDoubleTopic("I (Hood)").getEntry(Constants.HoodConstants.kI);
        public static DoubleEntry kD = hoodTable.getDoubleTopic("D (Hood)").getEntry(Constants.HoodConstants.kD);


        public static void init() {
            kManualPower.set(kManualPower.get());
            kAutoPower.set(kAutoPower.get());
            kDefaultAngle.set(kDefaultAngle.get());

            kP.set(kP.get());
            kI.set(kI.get());
            kD.set(kD.get());
        }
    }

    
    public static final class ShooterTable {
        static NetworkTable shooterTable = networkInstance.getTable("shooterTable");

        public static DoublePublisher velocity = shooterTable.getDoubleTopic("Shooter Velocity").publish();
        public static DoublePublisher powerGoal = shooterTable.getDoubleTopic("Shooter Velocity Goal").publish();

        public static DoubleEntry kTestPower = shooterTable.getDoubleTopic("Shooter Test Power").getEntry(Constants.ShooterConstants.kFixedPower);
        public static DoubleEntry kTestDistance = shooterTable.getDoubleTopic("Shooter Distance").getEntry(Constants.ShooterConstants.kTestDistance);

        public static DoubleEntry kTolerance = shooterTable.getDoubleTopic("Shooter Tolerance").getEntry(Constants.ShooterConstants.kTolerance);

        //slot 0 configs
        public static DoubleEntry kS = shooterTable.getDoubleTopic("S (Shooter)").getEntry(Constants.ShooterConstants.kS);; // Add 0.25 V output to overcome static friction
        public static DoubleEntry kV = shooterTable.getDoubleTopic("V (Shooter)").getEntry(Constants.ShooterConstants.kV);; // A velocity target of 1 rps results in 0.12 V output
        public static DoubleEntry kA = shooterTable.getDoubleTopic("A (Shooter)").getEntry(Constants.ShooterConstants.kA);; // An acceleration of 1 rps/s requires 0.01 V output
        public static DoubleEntry kP = shooterTable.getDoubleTopic("P (Shooter)").getEntry(Constants.ShooterConstants.kP);
        public static DoubleEntry kI = shooterTable.getDoubleTopic("I (Shooter)").getEntry(Constants.ShooterConstants.kI);
        public static DoubleEntry kD = shooterTable.getDoubleTopic("D (Shooter)").getEntry(Constants.ShooterConstants.kD);

        // set Motion Magic settings
        public static DoubleEntry kMotionMagicCruiseVelocity = shooterTable.getDoubleTopic("MM Velocity (Shooter)").getEntry(Constants.ShooterConstants.kMotionMagicCruiseVelocity);; // Target cruise velocity of 80 rps
        public static DoubleEntry kMotionMagicAcceleration = shooterTable.getDoubleTopic("MM Acceleration (Shooter)").getEntry(Constants.ShooterConstants.kMotionMagicAcceleration);; // Target acceleration of 160 rps/s (0.5 seconds)
        public static DoubleEntry kMotionMagicJerk = shooterTable.getDoubleTopic("MM Jerk (Shooter)").getEntry(Constants.ShooterConstants.kMotionMagicJerk);; // Target jerk of 1600 rps/s/s (0.1 seconds)

        public static void init() {
            velocity.set(0);

            kTestPower.set(kTestPower.get());
            kTestDistance.set(kTestDistance.get());

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
    
    public static void initialize() {
        SwerveTable.init();
        IntakeTable.init();
        HopperTable.init();
        IndexerTable.init();
        HoodTable.init();
        ShooterTable.init();
    }

    public static void smartDashboard() {
        field2d.setRobotPose(SwerveTable.robotPose.get());
        SmartDashboard.putData(field2d);
    }
}
