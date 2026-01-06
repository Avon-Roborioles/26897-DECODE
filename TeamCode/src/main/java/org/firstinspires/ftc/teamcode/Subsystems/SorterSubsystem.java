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

    public void setIntaking(boolean intaking) {
        this.isIntaking = intaking;
    }

    public void update(ElapsedTime timer) {
        // 1. If we aren't sorting a specific color AND we aren't in intake mode, STOP.
        if (targetColor == ArtifactColor.NOTHING && !isIntaking) {
            artifactServo.setPower(0);
            return;
        }

        int error = targetTicks - encoder.getCurrentPosition();
        int absError = Math.abs(error);

        // 2. Movement Logic (Same as before)
        if (absError > TICK_TOLERANCE) {
            artifactServo.setPower(error > 0 ? 0.2 : -0.2);
            sensorCheckTime = timer.milliseconds() + 400;
        } else if (absError > 50) {
            artifactServo.setPower(error > 0 ? 0.02 : -0.02);
            sensorCheckTime = timer.milliseconds() + 400;
        } else {
            // 3. We are at a slot!
            artifactServo.setPower(0);

            if(targetColor != ArtifactColor.NOTHING) {
                ArtifactColor seenColor = sensor.read();

                if (seenColor == ArtifactColor.NOTHING) {
                    targetTicks += TICKS_PER_SLOT;
                } else if (seenColor == targetColor) {
                    if(sensorCheckTime < 0){sensorCheckTime = timer.milliseconds() + 400;}
                    if(sensorCheckTime <= timer.milliseconds()) {
                        checkColorAndKick();
                    }
                } else {
                    targetTicks += TICKS_PER_SLOT;
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