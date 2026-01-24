package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ArtifactSensor;
import org.firstinspires.ftc.teamcode.ArtifactColor;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;

public class IntakeSubsystem {
    private DcMotor intake;
    private DcMotor kicker;
    private Servo artifactServo;
    private ArtifactSensor artifactSensor;

    private final double[] SLOT_POSITIONS = {0.2256, 0.3023, 0.3828};
    private int currentSlotIndex = 0;
    private final double SERVO_MOVE_DELAY = 150;
    private ElapsedTime timer = new ElapsedTime();


    // Indexing State Variables
    private boolean intakeSeekingEmpty = false;
    private int intakeSlotsChecked = 0;
    private boolean magazineFull = false;
    private double sensorCheckTime = -1;

<<<<<<< HEAD
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        kicker = hardwareMap.get(DcMotor.class, "kicker");
        artifactServo = hardwareMap.get(Servo.class, "sorterservo");
        artifactServo.setPosition(SLOT_POSITIONS[0]);
        artifactSensor = new ArtifactSensor(hardwareMap);
=======
    public IntakeSubsystem(DcMotor motor) {

        this.intakeMotor = intakeMotor;
    }
    public void runMotor() {
        intakeMotor.setPower(1.0);
>>>>>>> 07f70403f547835f99768f83acf1d0c358551ca3
    }

    // --- 1. CONTINUOUS INDEXING (Call in TeleOp loop) ---
    public void updateAutoIndex(double triggerValue, ArtifactColor targetColorState) {
        boolean intakeRunning = Math.abs(triggerValue) > 0.2;
        if (!intakeRunning) magazineFull = false;

        // A. Start searching for empty slot if we just picked something up
        if (intakeRunning && targetColorState == ArtifactColor.NOTHING && !intakeSeekingEmpty && !magazineFull) {
            if (artifactSensor.read() != ArtifactColor.NOTHING) {
                intakeSeekingEmpty = true;
                intakeSlotsChecked = 0;
                incrementSlot();
                sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
            }
        }

        // B. Handle seeking the next empty slot
        if (intakeSeekingEmpty) {
            if (timer.milliseconds() >= sensorCheckTime) {
                if (artifactSensor.read() == ArtifactColor.NOTHING) {
                    intakeSeekingEmpty = false;
                } else {
                    intakeSlotsChecked++;
                    if (intakeSlotsChecked >= 3) {
                        intakeSeekingEmpty = false;
                        magazineFull = true;
                    } else {
                        incrementSlot();
                        sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
                    }
                }
            }
        }

        // Apply power logic
        if (magazineFull) intake.setPower(0);
        else intake.setPower(triggerValue);
    }

    // --- 2. COLOR SEARCH AND KICK COMMAND ---
    public void searchAndKick(ArtifactColor target, LinearOpMode opMode) {
        int currentSlot = 0;

        // 1. Move to the first slot
        artifactServo.setPosition(SLOT_POSITIONS[0]);
        opMode.sleep(400);

        while (opMode.opModeIsActive()) {
            if (artifactSensor.read() == target) {
                // MATCH: Kick
                kicker.setPower(1.0);
                opMode.sleep(500);
                kicker.setPower(-1.0);
                opMode.sleep(150);
                kicker.setPower(0);
                return; // Exit the function once done
            } else {
                // WRONG COLOR: Move next
                currentSlot++;
                //if (currentSlot > SLOT_POSITIONS.length) return; // Exit if nothing found

                artifactServo.setPosition(SLOT_POSITIONS[currentSlot]);
                opMode.sleep((long) SERVO_MOVE_DELAY);
            }
        }
    }

    // --- 3. TRIPLE KICK SEQUENCE ---
    public Command tripleKickAndMove() {
        return new SequentialGroup(
                kickOnce(), moveAndWait(),
                kickOnce(), moveAndWait(),
                kickOnce(), moveAndWait()
        );
    }

    // --- HELPER METHODS ---
    public void incrementSlot() {
        currentSlotIndex = (currentSlotIndex + 1) % SLOT_POSITIONS.length;
        artifactServo.setPosition(SLOT_POSITIONS[currentSlotIndex]);
    }

    public Command kickOnce() {
        return new Command() {
            private double start;
            @Override public void start() { start = timer.milliseconds(); }
            @Override public void update() {
                double e = timer.milliseconds() - start;
                if (e < 500) kicker.setPower(1);
                else if (e < 600) kicker.setPower(-1);
                else kicker.setPower(0);
            }
            @Override public boolean isDone() { return (timer.milliseconds() - start) >= 600; }
        };
    }

    private Command moveAndWait() {
        return new Command() {
            private double end;
            @Override public void start() { incrementSlot(); end = timer.milliseconds() + SERVO_MOVE_DELAY; }
            @Override public boolean isDone() { return timer.milliseconds() >= end; }
        };
    }
}