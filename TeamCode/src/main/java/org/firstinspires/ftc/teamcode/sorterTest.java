package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Sorter Test")
public class sorterTest extends LinearOpMode {
    private DcMotor kicker;
    private ArtifactSensor artifactSensor;
    private Servo artifactServo;
    private double servoPosition = 0.1; // Current position of the servo
    private ArtifactColor targetColor = ArtifactColor.NOTHING;

    private ElapsedTime timer = new ElapsedTime();
    private boolean lastA = false;
    private boolean lastB = false;
    private double sensorCheckTime = -1;
    private double moveFinishTime = 0;

    @Override
    public void runOpMode() {
        artifactServo = hardwareMap.get(Servo.class, "sorterservo");
        artifactServo.setPosition(0.285);
        artifactSensor = new ArtifactSensor(hardwareMap);
        kicker = hardwareMap.get(DcMotor.class, "kicker");

        telemetry.addLine("A: Seek Green | B: Seek Purple | X: Reset");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 2. Sorting
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;

            if (currentA && !lastA) {
                targetColor = ArtifactColor.GREEN;
                servoPosition = 0.285;
                sensorCheckTime = timer.milliseconds() + 400; // Schedule a check 400ms from now
            } else if (currentB && !lastB) {
                targetColor = ArtifactColor.PURPLE;
                servoPosition = 0.285;
                sensorCheckTime = timer.milliseconds() + 400;
            }

            lastA = currentA;
            lastB = currentB;

            if (targetColor != ArtifactColor.NOTHING) {
                double currentTime = timer.milliseconds();


                // 2. Wait for the "Sensor Check Time"
                // We don't need moveFinishTime because the servo moves to the angle instantly
                if (currentTime < sensorCheckTime) {
                    artifactServo.setPosition(servoPosition);
                } else {
                    // 3. The wait is over. Read the sensor.
                    if (artifactSensor.read() == targetColor) {
                        // SUCCESS: Found it!
                        kicker.setPower(1);
                        sleep(500);
                        kicker.setPower(-1); // Pull kicker back
                        sleep(200);
                        kicker.setPower(0);

                        targetColor = ArtifactColor.NOTHING; // Search finished
                    } else {
                        // 4. FAIL: Move to next slot (0 -> 0.33 -> 0.66 -> 0)
                        servoPosition += 0.275;
                        if (servoPosition > 0.9) servoPosition = 0.285; // Reset if at end

                        artifactServo.setPosition(servoPosition);

                        // Schedule the next check (Wait 800ms for movement + sensor refresh)
                        sensorCheckTime = currentTime + 2000;
                    }
                }
            }

            telemetry.addData("Targeting", targetColor);
            telemetry.update();
        }
    }
}
