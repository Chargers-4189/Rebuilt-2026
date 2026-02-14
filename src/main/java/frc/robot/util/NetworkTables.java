package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.Constants;

public class NetworkTables {
    static NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();

    public static final class ShooterTable {
        static NetworkTable shooterTable = networkInstance.getTable("shootTable");

        public static DoubleEntry kPOWER = shooterTable.getDoubleTopic("kPOWER").getEntry(Constants.ShooterConstants.kTEST_POWER);

        public static DoubleEntry kDISTANCE = shooterTable.getDoubleTopic("kDISTANCE").getEntry(Constants.ShooterConstants.kDISTANCE);

        public static DoubleEntry kFUEL_NUM = shooterTable.getDoubleTopic("kFUEL_NUM").getEntry(Constants.ShooterConstants.kFUEL_NUM);
    }

    public static final class HoodTable {
        static NetworkTable hoodTable = networkInstance.getTable("hoodTable");

        public static DoubleEntry kANGLE = hoodTable.getDoubleTopic("kANGLE").getEntry(Constants.HoodConstants.kANGLE);
    }

    public static void initialize() {
        ShooterTable.kPOWER.set(ShooterTable.kPOWER.get());
        HoodTable.kANGLE.set(HoodTable.kANGLE.get());
        ShooterTable.kFUEL_NUM.set(ShooterTable.kFUEL_NUM.get());
        ShooterTable.kDISTANCE.set(ShooterTable.kDISTANCE.get());
    }
}
