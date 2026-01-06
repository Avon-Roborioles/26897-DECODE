package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import java.util.List;

@TeleOp(name = "Test Teleop")
public class TestTeleOp extends LinearOpMode {
    private MecanumDrivetrain drive;
    private ShooterSubsystem shooter;
    private CRServo artifactServo;
    private ArtifactSensor artifactSensor;
    private DcMotor kicker;
    private DcMotor intake;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime intaketimer = new ElapsedTime();

    private boolean lastA, lastB;
    private ArtifactColor targetColor = ArtifactColor.NOTHING;
    private double sensorCheckTime = -1;

    private DcMotorEx artifactEncoder;
    private int targetTicks = 0;
    private final double MAX_SORT_POWER = 0.2;
    private final int TICKS_PER_SLOT = 2710;    // 8192 / 3 slots
    private final int TICK_TOLERANCE = 1670;

    private boolean intakeSeekingEmpty = false;
    private int intakeSlotsChecked = 0;



    @Override
    public void runOpMode() {
        drive = new MecanumDrivetrain(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");

        artifactServo = hardwareMap.get(CRServo.class, "sorterservo");
        artifactEncoder = hardwareMap.get(DcMotorEx.class, "backLeft"); // Use same name as the encoder port
        artifactEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        artifactEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        artifactSensor = new ArtifactSensor(hardwareMap);

        // Kicking
        kicker = hardwareMap.get(DcMotor.class, "kicker");

        telemetry.addLine("A: Seek Green | B: Seek Purple | Y: Reset | X: Rev Intake");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Drive
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // 2. Shooter & Tracking
            shooter.updateTracking(telemetry);
            shooter.runAutoVelocity();
            if (gamepad1.right_bumper) {
                shooter.adjustMultiplier(0.01);
            }
            if (gamepad1.left_bumper) {
                shooter.adjustMultiplier(-0.01);
            }


            // 2. SORTING LOGIC
            boolean intakeRunning = Math.abs(gamepad1.left_trigger) > 0.2;

// START EMPTY-SLOT SEARCH
            if (intakeRunning &&
                    targetColor == ArtifactColor.NOTHING &&
                    !intakeSeekingEmpty) {

                intakeSeekingEmpty = true;
                intakeSlotsChecked = 0;
                sensorCheckTime = timer.milliseconds() + 400;
            }


            if (intakeSeekingEmpty && targetColor == ArtifactColor.NOTHING) {

                int currentTicks = artifactEncoder.getCurrentPosition();
                int error = targetTicks - currentTicks;

                // Move to target slot
                if (Math.abs(error) > TICK_TOLERANCE) {
                    artifactServo.setPower(error > 0 ? MAX_SORT_POWER : -MAX_SORT_POWER);
                }
                // At slot → check
                else {
                    artifactServo.setPower(0);

                    if (timer.milliseconds() >= sensorCheckTime) {

                        // EMPTY SLOT FOUND
                        if (artifactSensor.read() == ArtifactColor.NOTHING) {
                            intakeSeekingEmpty = false;
                            // stay here and let intake load
                        }
                        // SLOT OCCUPIED → STEP ONE SLOT
                        else {
                            intakeSlotsChecked++;

                            if (intakeSlotsChecked >= 3) {
                                // All slots full
                                intake.setPower(0);
                                intakeSeekingEmpty = false;
                            } else {
                                targetTicks += TICKS_PER_SLOT; // <-- SAME AS YOUR SORTER
                                sensorCheckTime = timer.milliseconds() + 500;
                            }
                        }
                    }
                }
            }

            if (!intakeSeekingEmpty &&
                    intakeRunning &&
                    artifactSensor.read() != ArtifactColor.NOTHING &&
                    targetColor == ArtifactColor.NOTHING) {

                artifactServo.setPower(0);
            }



            //Sorting

            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;

            if (currentA && !lastA) {
                targetColor = ArtifactColor.GREEN;
                targetTicks = artifactEncoder.getCurrentPosition();
                sensorCheckTime = timer.milliseconds() + 500;
            } else if (currentB && !lastB) {
                targetColor = ArtifactColor.PURPLE;
                targetTicks = artifactEncoder.getCurrentPosition();
                sensorCheckTime = timer.milliseconds() + 500;
            }
            lastA = currentA;
            lastB = currentB;

            if (targetColor != ArtifactColor.NOTHING) {
                int currentTicks = artifactEncoder.getCurrentPosition();
                int error = targetTicks - currentTicks;
                int absError = Math.abs(error);

                // STEP 1: Fast movement if we are far away
                if (absError > TICK_TOLERANCE) { // 1670+ ticks away
                    double power = (error > 0) ? MAX_SORT_POWER : -MAX_SORT_POWER;
                    artifactServo.setPower(power);
                    sensorCheckTime = timer.milliseconds() + 600;
                }
                // STEP 2: Final Stop (Within 50 ticks)
                else {
                    artifactServo.setPower(0);
                    if (timer.milliseconds() >= sensorCheckTime) {
                        if (artifactSensor.read() == targetColor) {
                            kicker.setPower(1);
                            sleep(500);
                            kicker.setPower(-1);
                            sleep(200);
                            kicker.setPower(0);
                            targetColor = ArtifactColor.NOTHING;
                            intake.setPower(0);
                        } else {
                            // Wrong color, move to next slot
                            targetTicks += TICKS_PER_SLOT;
                            sensorCheckTime = timer.milliseconds() + 600;
                        }
                    }
                }
            } else {
                artifactServo.setPower(0);
            }


            // 4. INTAKE & KICKER CONTROL


            // MANUAL KICKER (Shooting)
            if (gamepad1.right_trigger > 0.2) {
                kicker.setPower(1);
            } else {
                kicker.setPower(0);
            }

            // INTAKE
            double intakePower = gamepad1.left_trigger;

// If sorter logic has stopped intake (magazine full), do nothing
            if (!intakeSeekingEmpty && targetColor == ArtifactColor.NOTHING && intakePower == 0) {
                intake.setPower(0);
            }
            else if (gamepad1.x) {
                intake.setPower(intakePower);   // Outtake / reverse always allowed
            }
            else {
                intake.setPower(-intakePower);  // Normal intake
            }

        }

        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Intake Seeking", intakeSeekingEmpty);
        telemetry.update();
    }
}