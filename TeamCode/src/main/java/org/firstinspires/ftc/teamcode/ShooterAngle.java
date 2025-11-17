package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Testing the Shooter Angle", group = "Limelight")
public class ShooterAngle extends LinearOpMode {

    // === VISION CONSTANTS AND CONFIGURATION ===
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final static int TARGET_APRILTAG_ID = 2; // Change this to the ID of the tag you are targeting (e.g., 2, 4, etc.)
    private final static String CAMERA_NAME = "Limelight"; // Name of your camera in the config file

    // === SERVO CONFIGURATION ===
    private Servo aimServo = null;
    private final static String SERVO_NAME = "tiltservo";

    // === SERVO RANGE MAPPING (MUST BE TUNED!) ===
    // Use the actual range from the tag's pose data (in inches/mm, based on your AprilTag size config)
    private final static double MIN_DISTANCE_INCHES = 12.0; // Closest operational distance
    private final static double MAX_DISTANCE_INCHES = 60.0; // Farthest operational distance

    // Servo Limits
    private final static double SERVO_MIN_POS = 0.0;
    private final static double SERVO_MAX_POS = 1.0;

    // Set to true if further distance should mean a lower servo position (smaller number)
    private final static boolean REVERSE_MAPPING = true;

    @Override
    public void runOpMode() throws InterruptedException {

        // HARDWARE INITIALIZATION
        try {
            aimServo = hardwareMap.get(Servo.class, SERVO_NAME);
            telemetry.addData("Servo", "Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not find servo: " + SERVO_NAME);
            aimServo = null;
        }

        // VISION INITIALIZATION
        initVision();

        telemetry.addData("Vision Status", "VisionPortal Initialized");
        telemetry.addData("Instructions", "Ensure AprilTag ID " + TARGET_APRILTAG_ID + " is visible.");
        telemetry.update();

        waitForStart();

        // --- MAIN CONTROL LOOP ---
        while (opModeIsActive()) {

            // 1. GET APRILTAG DETECTIONS
            AprilTagDetection targetDetection = findTargetDetection();

            if (targetDetection != null) {
                // The ftcPose.range is the distance from the camera to the center of the tag
                // This value is in INCHES if you used the default FTC AprilTag configuration.
                double distance = targetDetection.ftcPose.range;

                // 2. MAP DISTANCE TO SERVO POSITION
                double targetServoPosition = distanceToServoPos(distance);

                // 3. APPLY SERVO POSITION
                if (aimServo != null) {
                    aimServo.setPosition(targetServoPosition);
                }

                // 4. TELEMETRY OUTPUT
                telemetry.addData("Status", "Tag Found (ID: " + targetDetection.id + ")");
                telemetry.addData("Distance (Range)", "%.2f in", distance);
                telemetry.addData("Elevation", "%.2f deg", targetDetection.ftcPose.elevation);
                telemetry.addData("Target Servo Pos", "%.3f", targetServoPosition);

            } else {
                // Target not found - Set servo to a safe default position
                if (aimServo != null) {
                    // Default to one extreme when no target is seen
                    aimServo.setPosition(REVERSE_MAPPING ? SERVO_MIN_POS : SERVO_MAX_POS);
                }
                telemetry.addData("Status", "Target Not Found");
                telemetry.addData("Servo", "Set to default (%.3f)", aimServo != null ? aimServo.getPosition() : 0.0);
            }

            telemetry.update();
        }

        // --- CLEANUP ---
        visionPortal.close();
    }

    /**
     * Initializes the FTC VisionPortal and AprilTagProcessor.
     */
    private void initVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Build the Vision Portal, using the AprilTagProcessor.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * Finds the target AprilTag detection (by ID).
     * @return The target detection object, or null if not found.
     */
    private AprilTagDetection findTargetDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == TARGET_APRILTAG_ID) {
                return detection;
            }
        }
        return null; // Target tag not found
    }



    private double distanceToServoPos(double distance) {
        // Step 1: Normalize the distance (map to 0.0 to 1.0)
        double normalizedDistance = (distance - MIN_DISTANCE_INCHES) /
                (MAX_DISTANCE_INCHES - MIN_DISTANCE_INCHES);

        // Step 2: Handle reversal
        double mappedValue = normalizedDistance;
        if (REVERSE_MAPPING) {
            // Invert the normalized value: 0.0 (close) -> 1.0; 1.0 (far) -> 0.0
            mappedValue = 1.0 - normalizedDistance;
        }

        // Step 3: Scale to the servo's physical limits (SERVO_MIN_POS to SERVO_MAX_POS)
        double servoPosition = (mappedValue * (SERVO_MAX_POS - SERVO_MIN_POS)) + SERVO_MIN_POS;

        // Step 4: Clamp to the absolute safe range [0.0, 1.0]
        return Range.clip(servoPosition, 0.0, 1.0);
    }
}