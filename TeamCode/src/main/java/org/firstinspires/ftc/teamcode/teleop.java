package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo; // Changed from CRServo
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(name = "teleop with intakesusbsystem")
public class teleop extends LinearOpMode {
    private MecanumDrivetrain drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

//    // CHANGED: Position based servo
//    private ServoImplEx artifactServo;
//    private boolean lastr;
//
//    private ArtifactSensor artifactSensor;
//    private DcMotor kicker;
//    private DcMotor intake;
//    private ElapsedTime timer = new ElapsedTime();
//
//    private Servo indicatorLight;
//
//    private ArtifactColor targetColor = ArtifactColor.NOTHING;
//    private double sensorCheckTime = -1;
//
//    // --- NEW SORTING VARIABLES ---
//    // UPDATE THESE VALUES using the telemetry readouts from the D-Pad calibration
//    private final double[] SLOT_POSITIONS = {0.2256,0.3023,0.3828,0.4589};
//    private int currentSlotIndex = 0; // Tracks which of the 3 slots is active
//
//    // Time to wait for servo to physically move before reading sensor (milliseconds)
//    private final double SERVO_MOVE_DELAY = 700;
//
//    private boolean intakeSeekingEmpty = false;
//    private int intakeSlotsChecked = 0;
//    private boolean magazineFull = false;
//
//    // For manual calibration
//    private double manualPosition = 0.5;

    @Override
    public void runOpMode() {
        drive = new MecanumDrivetrain(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
//        intake = hardwareMap.get(DcMotor.class, "intake");
//
//        // CHANGED: Mapping to standard Servo
//        artifactServo = (ServoImplEx) hardwareMap.get(Servo.class, "sorterservo");
//        indicatorLight = hardwareMap.get(Servo.class, "colorlight");
//
//        // Removed artifactEncoder setup (not needed for Position Servo)
//
//        artifactSensor = new ArtifactSensor(hardwareMap);
//        kicker = hardwareMap.get(DcMotor.class, "kicker");
        intake = new IntakeSubsystem(hardwareMap);


        telemetry.addLine("A: Seek Green | B: Seek Purple | Y: Reset | X: Rev Intake");
        telemetry.addLine("D-Pad Up/Down: Manually Adjust Servo Position");
        telemetry.update();



        waitForStart();

//        // Set initial position
//        artifactServo.setPosition(SLOT_POSITIONS[currentSlotIndex]);

        while (opModeIsActive()) {
            // 1. Drive
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // 2. Shooter & Tracking
            shooter.updateTracking(telemetry);
            shooter.runAutoVelocity(telemetry);
            if (gamepad1.rightBumperWasPressed()) shooter.increaseVelocity();
            if (gamepad1.leftBumperWasPressed()) shooter.decreaseVelocity();


//            // MANUAL CALIBRATION (To find your exact positions)
//            if (gamepad1.dpad_up) {
//                manualPosition += 0.001;
//                // Clamp to safe range
//                if (manualPosition > 1.0) manualPosition = 1.0;
//                artifactServo.setPosition(manualPosition);
//            } else if (gamepad1.dpad_down) {
//                manualPosition -= 0.001;
//                if (manualPosition < 0.0) manualPosition = 0.0;
//                artifactServo.setPosition(manualPosition);
//            }


//            boolean intakeRunning = Math.abs(gamepad2.left_trigger) > 0.2;
//
//            if (!intakeRunning) {
//                magazineFull = false;
//            }
//
//            // A. START SEARCHING (Trigger pressed, not full, not already seeking)
//            if (intakeRunning && targetColor == ArtifactColor.NOTHING && !intakeSeekingEmpty && !magazineFull) {
//                if(artifactSensor.read() != ArtifactColor.NOTHING) {
//                    intakeSeekingEmpty = true;
//                    intakeSlotsChecked = 0;
//
//                    // Move to next slot
//                    incrementSlot();
//                    sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
//                }
//            }
//
//            // B. HANDLE EMPTY SLOT SEEKING
//            if (intakeSeekingEmpty) {
//                // Wait for servo to arrive (using timer)
//                if (timer.milliseconds() >= sensorCheckTime) {
//                    // We have arrived. Check sensor.
//                    if (artifactSensor.read() == ArtifactColor.NOTHING) {
//                        // Found empty slot! Stop seeking.
//                        intakeSeekingEmpty = false;
//                    } else {
//                        // Slot occupied, try next
//                        intakeSlotsChecked++;
//                        if (intakeSlotsChecked >= 3) {
//                            intakeSeekingEmpty = false;
//                            magazineFull = true;
//                        } else {
//                            incrementSlot();
//                            sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
//                        }
//                    }
//                }
//            }
//
//            // C. HANDLE COLOR SORTING (A/B Buttons)
//            if (gamepad2.a) {
//                targetColor = ArtifactColor.GREEN;
//                // Note: Position servos reset immediately, so we just check the current pos
//                artifactServo.setPosition(SLOT_POSITIONS[0]);
//                currentSlotIndex = 0;
//                sensorCheckTime = timer.milliseconds() + 400;
//            }
//            if (gamepad2.b) {
//                targetColor = ArtifactColor.PURPLE;
//                artifactServo.setPosition(SLOT_POSITIONS[0]);
//                currentSlotIndex = 0;
//                sensorCheckTime = timer.milliseconds() + 400;
//            }
//
//            if (targetColor != ArtifactColor.NOTHING) {
//                // Wait for any previous movement to settle
//                if (timer.milliseconds() >= sensorCheckTime) {
//                    if (artifactSensor.read() == targetColor) {
//                        // MATCH FOUND: Kick
//                        kicker.setPower(1);
//                        sleep(500);
//                        kicker.setPower(-1);
//                        sleep(100);
//                        kicker.setPower(0);
//
//                        targetColor = ArtifactColor.NOTHING;
//                        intake.setPower(0);
//                    } else {
//                        // WRONG COLOR: Next slot
//                        incrementSlot();
//                        sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
//                    }
//                }
//            }
            // Inside your TeleOp Loop
            ArtifactColor currentTarget = ArtifactColor.NOTHING; // Local state tracking

// 1. Handle Auto-Indexing (Finding empty slots)
            intake.updateAutoIndex(gamepad2.left_trigger, currentTarget);

// 2. Handle Color Sorting Trigger
            if (gamepad2.a) {
                intake.searchAndKick(ArtifactColor.GREEN, this);
            }
            if (gamepad2.b) {
                intake.searchAndKick(ArtifactColor.PURPLE, this);
            }

            if(gamepad2.right_trigger > 0.1) {
                intake.kickOnce();
            }

// 3. Handle Triple Shot
            if (gamepad2.y) {
                intake.tripleKickAndMove().schedule();
            }

//            if(artifactSensor.read() == ArtifactColor.GREEN) {
//                indicatorLight.setPosition(0.500);
//            } else if(artifactSensor.read() == ArtifactColor.PURPLE) {
//                indicatorLight.setPosition(0.722);
//            } else {
//                indicatorLight.setPosition(0.388);
//            }

//            // Intake Power Logic
//            if (magazineFull && !gamepad2.x) {
//                intake.setPower(0);
//            } else if (gamepad2.x) {
//                shooter.setZero();
//                intake.setPower(-gamepad2.left_trigger);
//            } else {
//                shooter.setZero();
//                intake.setPower(gamepad2.left_trigger);
//            }
//
//            if(gamepad2.right_trigger > 0.5) {
//                kicker.setPower(1);
//            } else {
//                kicker.setPower(0);
//            }
//
//            if(gamepad2.y) {
//                incrementSlot();
//            }
//            if(gamepad1.dpad_left) {
//                artifactServo.setPosition(0);
//            }


            boolean currentr = gamepad1.dpad_right;

//            if (currentr && !lastr) {
//                incrementSlot();
//            }

            if(gamepad1.x) {
                shooter.mid();
            }


            // TELEMETRY UPDATES
//            telemetry.addData("State", magazineFull ? "FULL" : (intakeSeekingEmpty ? "INDEXING" : "READY"));
//            telemetry.addLine("-----------------------------");
//            // This displays the last command sent to the servo
//            telemetry.addData("SERVO POS", artifactServo.getPosition());
//            telemetry.addData("Current Slot Index", currentSlotIndex);
//            telemetry.update();
        }
    }

    // Helper method to cycle through 3 slots
//    private void incrementSlot() {
//        currentSlotIndex++;
//        if (currentSlotIndex >= SLOT_POSITIONS.length) {
//            currentSlotIndex = 0;
//        }
//        double cur = artifactServo.getPosition();
//        artifactServo.setPosition(SLOT_POSITIONS[currentSlotIndex]);
//        // Update manual var so D-Pad starts from here if touched
//        manualPosition = SLOT_POSITIONS[currentSlotIndex];
//    }
}