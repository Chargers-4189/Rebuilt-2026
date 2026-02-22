package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;

public class NetworkTables {
    static NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();

    public static final class SwerveTable {
        static NetworkTable swerveTable = networkInstance.getTable("swerveTable");

        public static StructPublisher<Pose2d> robotPose = swerveTable.getStructTopic("Robot Position", Pose2d.struct).publish();
        public static StructPublisher<Rotation3d> gyroRotation = swerveTable.getStructTopic("Gyro Rotation", Rotation3d.struct).publish();
        public static StructPublisher<Pose2d> aprilTagPose = swerveTable.getStructTopic("April Tag Pose", Pose2d.struct).publish();
        public static DoublePublisher hubDistance = swerveTable.getDoubleTopic("Hub Distance").publish();

        public static DoubleEntry kP = swerveTable.getDoubleTopic("P (Swerve)").getEntry(Constants.SwerveConstants.kP);
        public static DoubleEntry kI = swerveTable.getDoubleTopic("I (Swerve)").getEntry(Constants.SwerveConstants.kI);
        public static DoubleEntry kD = swerveTable.getDoubleTopic("D (Swerve)").getEntry(Constants.SwerveConstants.kD);

        public static void init() {
            kP.set(kP.get());
            kI.set(kI.get());
            kD.set(kD.get());
        }
    }

    public static final class IntakeTable {
        static NetworkTable intakeTable = networkInstance.getTable("intakeTable");

        public static DoubleEntry kIntakeSpeed = intakeTable.getDoubleTopic("Intake Power").getEntry(Constants.IntakeConstants.kIntakeSpeed);

        public static void init() {
            kIntakeSpeed.set(kIntakeSpeed.get());
        }
    }

    public static final class HopperTable {
        static NetworkTable hopperTable = networkInstance.getTable("hopperTable");

        public static DoubleEntry kPower = hopperTable.getDoubleTopic("Hopper Power").getEntry(Constants.HopperConstants.kPower);

        public static void init() {
            kPower.set(kPower.get());
        }
    }
    
    
    public static final class IndexerTable {
        static NetworkTable indexerTable = networkInstance.getTable("indexerTable");

        public static DoubleEntry kPower = indexerTable.getDoubleTopic("Indexer Power").getEntry(Constants.IndexerConstants.kPower);

        public static void init() {
            kPower.set(kPower.get());
        }
    }

    public static final class HoodTable {
        static NetworkTable hoodTable = networkInstance.getTable("hoodTable");

        public static DoublePublisher hoodEncoder = hoodTable.getDoubleTopic("Hood Encoder").publish();

        public static DoubleEntry kManualPower = hoodTable.getDoubleTopic("Hood Manual Power").getEntry(Constants.HoodConstants.kManualPower);
        public static DoubleEntry kAutoPower = hoodTable.getDoubleTopic("Hood Auto Power").getEntry(Constants.HoodConstants.kAutoPower);

        public static DoubleEntry kTestAngle = hoodTable.getDoubleTopic("Hood Angle").getEntry(Constants.HoodConstants.kTestAngle);
        public static DoubleEntry kP = hoodTable.getDoubleTopic("P (Hood)").getEntry(Constants.HoodConstants.kP);
        public static DoubleEntry kI = hoodTable.getDoubleTopic("I (Hood)").getEntry(Constants.HoodConstants.kI);
        public static DoubleEntry kD = hoodTable.getDoubleTopic("D (Hood)").getEntry(Constants.HoodConstants.kD);

        public static void init() {
            kManualPower.set(kManualPower.get());
            kTestAngle.set(kTestAngle.get());
            kP.set(kP.get());
            kI.set(kI.get());
            kD.set(kD.get());
        }
    }

    
    public static final class ShooterTable {
        static NetworkTable shooterTable = networkInstance.getTable("shooterTable");

        public static DoublePublisher velocity = shooterTable.getDoubleTopic("Shooter Velocity").publish();

        public static DoubleEntry kPower = shooterTable.getDoubleTopic("Shooter Test Power").getEntry(Constants.ShooterConstants.kTestPower);
        public static DoubleEntry kDistance = shooterTable.getDoubleTopic("Shooter Distance").getEntry(Constants.ShooterConstants.kTestDistance);

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

            kPower.set(kPower.get());
            kDistance.set(kDistance.get());

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
}
