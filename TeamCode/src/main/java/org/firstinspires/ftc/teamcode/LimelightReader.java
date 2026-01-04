//package org.firstinspires.ftc.teamcode;
//
//import edu.wpi.first.wpilibj.networktables.NetworkTable;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
///**
// * A helper class to manage reading data from the Limelight camera
// * via NetworkTables, which is standard for Limelight communication.
// * * NOTE: This requires the NetworkTable dependency in your build.gradle file.
// */
//public class LimelightReader {
//
//    private NetworkTable limelightTable;
//    private LinearOpMode opMode;
//
//    // Limelight's default NetworkTable name
//    private static final String LIMELIGHT_NAME = "limelight";
//
//    /**
//     * Constructor. Initializes the NetworkTable instance.
//     * @param opMode The parent OpMode for telemetry access.
//     */
//    public LimelightReader(LinearOpMode opMode) {
//        this.opMode = opMode;
//        try {
//            // Get the default instance of NetworkTables and the Limelight table
//            // You may need to call NetworkTable.setClientMode() or similar if this fails.
//            // Ensure the Control Hub is configured to communicate with the Limelight's IP.
//            limelightTable = NetworkTable.getTable(LIMELIGHT_NAME);
//            opMode.telemetry.addData("Limelight", "NetworkTable Initialized");
//        } catch (Exception e) {
//            opMode.telemetry.addData("Limelight Error", "Failed to get NetworkTable: " + e.getMessage());
//            limelightTable = null;
//        }
//    }
//
//    /**
//     * Checks if the Limelight has a valid target (tv = 1).
//     * @return true if target is seen, false otherwise.
//     */
//    public boolean isTargetValid() {
//        if (limelightTable == null) return false;
//        // Limelight reports 1.0 if target is valid, 0.0 otherwise
//        return limelightTable.getNumber("tv", 0.0).doubleValue() == 1.0;
//    }
//
//    /**
//     * Gets the vertical offset of the target (ty).
//     * @return The vertical offset in degrees.
//     */
//    public double getTy() {
//        if (limelightTable == null) return 0.0;
//        // Gets 'ty', the vertical offset from the crosshair to the target center
//        return limelightTable.getNumber("ty", 0.0).doubleValue();
//    }
//
//    /**
//     * Gets the horizontal offset of the target (tx).
//     * @return The horizontal offset in degrees.
//     */
//    public double getTx() {
//        if (limelightTable == null) return 0.0;
//        // Gets 'tx', the horizontal offset from the crosshair to the target center
//        return limelightTable.getNumber("tx", 0.0).doubleValue();
//    }
//}