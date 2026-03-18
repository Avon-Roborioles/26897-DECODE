//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.Servo; // Changed from CRServo
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
//import org.firstinspires.ftc.teamcode.Subsystems.BlueShooterSubsystem;
//
//@TeleOp(name = "Blue Teleop")
//public class BlueTeleop extends LinearOpMode {
//    private double kickStartTime = -1; // -1 means we are not currently kicking
//
//    private MecanumDrivetrain drive;
//    private BlueShooterSubsystem shooter;
//
//    private ElapsedTime timer = new ElapsedTime();
//
//    // CHANGED: Position based servo
//    private ServoImplEx artifactServo;
//    private boolean lastr;
//
//    private ArtifactSensor artifactSensor;
//    private DcMotor kicker;
//    private DcMotor intake;
//
//    private Servo indicatorLight;
//
//    private ArtifactColor targetColor = ArtifactColor.NOTHING;
//    private double sensorCheckTime = -1;
//
//    // --- NEW SORTING VARIABLES ---
//    // UPDATE THESE VALUES using the telemetry readouts from the D-Pad calibration
//    private final double[] SLOT_POSITIONS = {0.7048,0.7853,0.8658,0.9463};
//    private int currentSlotIndex = 0; // Tracks which of the 3 slots is active
//
//    // Time to wait for servo to physically move before reading sensor (milliseconds)
//    private final double SERVO_MOVE_DELAY = 1000;
//
//    private boolean intakeSeekingEmpty = false;
//    private int intakeSlotsChecked = 0;
//    private boolean magazineFull = false;
//
//    // For manual calibration
//    private double manualPosition = 0.5;
//
//
//    ElapsedTime kickTimer = new ElapsedTime();
//    boolean isKicking = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new MecanumDrivetrain(hardwareMap);
//        WaitCommand wait = new WaitCommand(1000);
//        shooter = new BlueShooterSubsystem(hardwareMap, telemetry);
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
//
//
//        telemetry.addLine("A: Seek Green | B: Seek Purple | Y: Reset | X: Rev Intake");
//        telemetry.addLine("D-Pad Up/Down: Manually Adjust Servo Position");
//        telemetry.update();
//
//
//
//        waitForStart();
//
//        // Set initial position
//        artifactServo.setPosition(SLOT_POSITIONS[currentSlotIndex]);
//
//        while (opModeIsActive()) {
//            // 1. Drive
//            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
//
//            // 2. Shooter & Tracking
//            shooter.updateTracking(telemetry);
//            shooter.runAutoVelocity(telemetry);
//            if (gamepad1.rightBumperWasPressed()) shooter.increaseVelocity();
//            if (gamepad1.leftBumperWasPressed()) shooter.decreaseVelocity();
//
//
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
//
//
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
//
//
//            // C. HANDLE COLOR SORTING (A/B Buttons)
//            if (gamepad2.a) {
//                targetColor = ArtifactColor.GREEN;
//                sensorCheckTime = timer.milliseconds() + 400;
//                ElapsedTime actionTimer = new ElapsedTime();
//            }
//            if (gamepad2.b) {
//                targetColor = ArtifactColor.PURPLE;
//                sensorCheckTime = timer.milliseconds() + 400;
//            }
//
//
//
//// 2. Inside your TeleOp loop:
//            if (!isKicking) {
//                // Look for the artifact only if we aren't already kicking
//                if (targetColor != ArtifactColor.NOTHING && timer.milliseconds() >= sensorCheckTime) {
//                    if (artifactSensor.read() == targetColor) {
//                        isKicking = true;
//                        kickTimer.reset(); // Start the sequence
//                    } else {
//                        incrementSlot();
//                        sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
//                    }
//                }
//            } else {
//                // 3. The Sequence (State Machine)
//                double elapsed = kickTimer.milliseconds();
//
//                if (elapsed < 500) {
//                    kicker.setPower(1);
//                } else if (elapsed < 600) {
//                    kicker.setPower(-1);
//                } else if (elapsed < 850) {
//                    kicker.setPower(0);
//                    intake.setPower(0);
//                } else {
//                    // Sequence finished! Reset for next time
//                    kicker.setPower(0);
//                    isKicking = false;
//                    targetColor = ArtifactColor.NOTHING;
//                }
//            }
//
////            if (targetColor != ArtifactColor.NOTHING) {
////                if (timer.milliseconds() >= sensorCheckTime) {
////                    if (timer.milliseconds() >= sensorCheckTime) {
////                        if (artifactSensor.read() == targetColor) {
////                            ElapsedTime actionTimer = new ElapsedTime();
////                            actionTimer.reset();
////                            double elapsed = actionTimer.milliseconds();
////                            // Sequence of events based on time instead of sleeps
////                            if (elapsed < 1000) {
////                                kicker.setPower(1);    // Kick out
////                            } else if (elapsed < 1100) {
////                                kicker.setPower(-1);   // Retract
////                            } else if (elapsed < 1350) {
////                                kicker.setPower(0);    // Wait for settle
////                                intake.setPower(0);
////                            }
////                            targetColor = ArtifactColor.NOTHING;
////                        } else {
////                            // WRONG COLOR: Next slot
////                            incrementSlot();
////                            sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
////                        }
////                    }
////                }
////            }
//
//            if(artifactSensor.read() == ArtifactColor.GREEN) {
//                indicatorLight.setPosition(0.500);
//            } else if(artifactSensor.read() == ArtifactColor.PURPLE) {
//                indicatorLight.setPosition(0.722);
//            } else {
//                indicatorLight.setPosition(0.388);
//            }
//
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
//
//            if(gamepad2.yWasPressed()) {
//                int ballcount = 0;
//                if (artifactSensor.read() == ArtifactColor.PURPLE || artifactSensor.read() == ArtifactColor.GREEN) {
//                    // MATCH FOUND: Kick
//                    kicker.setPower(1);
//                    sleep(500);
//                    kicker.setPower(-1);
//                    sleep(100);
//                    kicker.setPower(0);
//
//                    targetColor = ArtifactColor.NOTHING;
//                    intake.setPower(0);
//                } else {
//                    // WRONG COLOR: Next slot
//                    incrementSlot();
//                    ballcount = ballcount + 1;
//                    sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
//                }
//
//
//            }
//            if(gamepad1.dpad_left) {
//                artifactServo.setPosition(0);
//            }
//
//            if(gamepad1.x) {
//                shooter.mid();
//            }
//
//
//            boolean currentr = gamepad1.dpad_right;
//
//            if (currentr && !lastr) {
//                incrementSlot();
//            }
//
//            lastr = currentr;
//
//            // TELEMETRY UPDATES
//            telemetry.addData("State", magazineFull ? "FULL" : (intakeSeekingEmpty ? "INDEXING" : "READY"));
//            telemetry.addLine("-----------------------------");
//            // This displays the last command sent to the servo
//            telemetry.addData("SERVO POS", artifactServo.getPosition());
//            telemetry.addData("Current Slot Index", currentSlotIndex);
//            telemetry.update();
//        }
//    }
//
//    // Helper method to cycle through 3 slots
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
//}