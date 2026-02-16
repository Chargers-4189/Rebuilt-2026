package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.Constants;

public class NetworkTables {
    static NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();

    public static final class ShooterTable {
        static NetworkTable shooterTable = networkInstance.getTable("shootTable");

        public static DoublePublisher velocity = shooterTable.getDoubleTopic("Velocity").publish();


        public static DoubleEntry kPOWER = shooterTable.getDoubleTopic("kPOWER").getEntry(Constants.ShooterConstants.kTEST_POWER);

        public static DoubleEntry kDISTANCE = shooterTable.getDoubleTopic("kDISTANCE").getEntry(Constants.ShooterConstants.kDISTANCE);

        public static DoubleEntry kFUEL_NUM = shooterTable.getDoubleTopic("kFUEL_NUM").getEntry(Constants.ShooterConstants.kFUEL_NUM);

        public static DoubleEntry kSPEED = shooterTable.getDoubleTopic("kSPEED").getEntry(Constants.ShooterConstants.kSPEED);
        public static DoubleEntry kP = shooterTable.getDoubleTopic("kP").getEntry(Constants.ShooterConstants.kP);
        public static DoubleEntry kI = shooterTable.getDoubleTopic("kI").getEntry(Constants.ShooterConstants.kI);
        public static DoubleEntry kD = shooterTable.getDoubleTopic("kD").getEntry(Constants.ShooterConstants.kD);
        //slot 0 configs
        public static DoubleEntry kS = shooterTable.getDoubleTopic("kS").getEntry(Constants.ShooterConstants.kS);; // Add 0.25 V output to overcome static friction
        public static DoubleEntry kV = shooterTable.getDoubleTopic("kV").getEntry(Constants.ShooterConstants.kV);; // A velocity target of 1 rps results in 0.12 V output
        public static DoubleEntry kA = shooterTable.getDoubleTopic("kA").getEntry(Constants.ShooterConstants.kA);; // An acceleration of 1 rps/s requires 0.01 V output

        // set Motion Magic settings
        public static DoubleEntry MotionMagicCruiseVelocity = shooterTable.getDoubleTopic("MotionMagicCruiseVelocity").getEntry(Constants.ShooterConstants.MotionMagicCruiseVelocity);; // Target cruise velocity of 80 rps
        public static DoubleEntry MotionMagicAcceleration = shooterTable.getDoubleTopic("MotionMagicAcceleration").getEntry(Constants.ShooterConstants.MotionMagicAcceleration);; // Target acceleration of 160 rps/s (0.5 seconds)
        public static DoubleEntry MotionMagicJerk = shooterTable.getDoubleTopic("MotionMagicJerk").getEntry(Constants.ShooterConstants.MotionMagicJerk);; // Target jerk of 1600 rps/s/s (0.1 seconds)
    }

    public static final class HoodTable {
        static NetworkTable hoodTable = networkInstance.getTable("hoodTable");

        public static DoubleEntry kANGLE = hoodTable.getDoubleTopic("kANGLE").getEntry(Constants.HoodConstants.kANGLE);
        public static DoubleEntry kP = hoodTable.getDoubleTopic("kP").getEntry(Constants.HoodConstants.kP);
        public static DoubleEntry kI = hoodTable.getDoubleTopic("kI").getEntry(Constants.HoodConstants.kI);
        public static DoubleEntry kD = hoodTable.getDoubleTopic("kD").getEntry(Constants.HoodConstants.kD);
    }

    public static void initialize() {
        ShooterTable.velocity.set(0.0);
        ShooterTable.kPOWER.set(ShooterTable.kPOWER.get());
        HoodTable.kANGLE.set(HoodTable.kANGLE.get());
        ShooterTable.kS.set(ShooterTable.kS.get());
        ShooterTable.kV.set(ShooterTable.kV.get());
        ShooterTable.kA.set(ShooterTable.kA.get());
        ShooterTable.kP.set(ShooterTable.kP.get());
        ShooterTable.kI.set(ShooterTable.kI.get());
        ShooterTable.kD.set(ShooterTable.kD.get());
        ShooterTable.MotionMagicCruiseVelocity.set(ShooterTable.MotionMagicCruiseVelocity.get());
        ShooterTable.kFUEL_NUM.set(ShooterTable.kFUEL_NUM.get());
        ShooterTable.kDISTANCE.set(ShooterTable.kDISTANCE.get());
    }
}
