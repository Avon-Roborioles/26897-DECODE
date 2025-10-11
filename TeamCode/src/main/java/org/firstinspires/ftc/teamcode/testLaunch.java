package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Launch + Smooth Servo", group="TeleOp")
public class testLaunch extends OpMode {

    private DcMotor launchMotor;
    private Servo angleServo;

    // --- Motor control ---
    private int targetRPM = 1000;   // starting rpm
    private double motorPower = 0;  // persistent power value
    private final int RPM_STEP = 10;
    private final int MAX_RPM = 6000; // adjust to your motor specs

    // --- Servo control ---
    private double servoPos = 0.5;        // actual servo position
    private double targetServoPos = 0.5;  // joystick target
    private final double SERVO_STEP = 0.01;  // joystick sensitivity
    private final double SMOOTH_RATE = 0.02; // easing rate per loop

    @Override
    public void init() {
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        angleServo = hardwareMap.get(Servo.class, "angleServo");

        launchMotor.setPower(0);
        double startingPos = angleServo.getPosition();
        angleServo.setPosition(startingPos);

        // ensure servoPos starts where servo is
        servoPos = startingPos;
        targetServoPos = startingPos;
    }

    @Override
    public void loop() {
        // ---- Servo Control (smooth easing from stick input) ----
        targetServoPos -= gamepad1.left_stick_y * SERVO_STEP;
        targetServoPos = Math.max(0.0, Math.min(1.0, targetServoPos));

        if (servoPos < targetServoPos) {
            servoPos = Math.min(servoPos + SMOOTH_RATE, targetServoPos);
        } else if (servoPos > targetServoPos) {
            servoPos = Math.max(servoPos - SMOOTH_RATE, targetServoPos);
        }
        angleServo.setPosition(servoPos);

        // ---- Motor RPM Control (A/B buttons) ----
        if (gamepad1.a) {
            targetRPM += RPM_STEP;
            if (targetRPM > MAX_RPM) targetRPM = MAX_RPM;
        }
        if (gamepad1.b) {
            targetRPM -= RPM_STEP;
            if (targetRPM < 0) targetRPM = 0;
        }

        // Convert targetRPM into motor power (simple linear scale)
        motorPower = (double) targetRPM / MAX_RPM;
        motorPower = Math.min(1.0, Math.max(0.0, motorPower));

        if (gamepad1.right_trigger > 0.1) {
            launchMotor.setPower(motorPower);
        } else {
            launchMotor.setPower(0);
        }

        // ---- Telemetry ----
        telemetry.addData("Servo Pos", servoPos);
        telemetry.addData("Target Servo", targetServoPos);
        telemetry.addData("Motor Power", motorPower);
        telemetry.update();
    }
}
