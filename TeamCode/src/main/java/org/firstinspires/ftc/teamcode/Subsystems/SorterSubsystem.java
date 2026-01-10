package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ArtifactColor;
import org.firstinspires.ftc.teamcode.ArtifactSensor;

public class SorterSubsystem {
    private CRServo artifactServo;
    private DcMotorEx encoder;
    private ArtifactSensor sensor;
    private DcMotor kicker;

    private int targetTicks = 0;
    private ArtifactColor targetColor = ArtifactColor.NOTHING;
    private double sensorCheckTime = -1;
    private boolean isIntaking = false;

    private final int TICKS_PER_SLOT = 2710;
    private final int TICK_TOLERANCE = 1670;

    public SorterSubsystem(HardwareMap hardwareMap) {
        artifactServo = hardwareMap.get(CRServo.class, "sorterservo");
        encoder = hardwareMap.get(DcMotorEx.class, "backLeft");
        sensor = new ArtifactSensor(hardwareMap);
        kicker = hardwareMap.get(DcMotor.class, "kicker");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startSorting(ArtifactColor color) {
        targetColor = color;
        targetTicks = encoder.getCurrentPosition() + TICKS_PER_SLOT;
    }


    public void update(ElapsedTime timer) {
        // 1. If we aren't sorting a specific color AND we aren't in intake mode, STOP.
        if (targetColor == ArtifactColor.NOTHING && !isIntaking) {
            artifactServo.setPower(0);
            return;
        }

        int currentTicks = artifactEncoder.getCurrentPosition();
        int error = targetTicks - currentTicks;
        int absError = Math.abs(error);

        // STEP 1: Fast movement if we are far away
        if (absError > TICK_TOLERANCE) { // 1670+ ticks away
            double power = (error > 0) ? MAX_SORT_POWER : -MAX_SORT_POWER;
            artifactServo.setPower(power);
            sensorCheckTime = timer.milliseconds() + 600;
        }
        // STEP 2: Final Stop
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
    }

    private void checkColorAndKick() {
        if (sensor.read() == targetColor) {
            kicker.setPower(1);
            // In a real subsystem, you'd use a timer instead of sleep,
            // but keeping it simple for now.
            targetColor = ArtifactColor.NOTHING;
        } else {
            targetTicks += TICKS_PER_SLOT;
        }
    }

    public void manualKicker(double power) { kicker.setPower(power); }
}