package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="testLaunch and smooth", group="TeleOp")
public class testLaunch extends OpMode {

    private DcMotor launchMotor;
    private Servo angleServo;

    double motorPower = 0;         // adjustable by right stick
    double servoPos = 0.5;         // actual servo position
    double targetServoPos = 0.5;   // where we *want* the servo to go

    @Override
    public void init() {
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        angleServo = hardwareMap.get(Servo.class, "angleServo");

        launchMotor.setPower(0);
        double startingPos = angleServo.getPosition();
        angleServo.setPosition(startingPos);
    }

    @Override
    public void loop() {
        // ---- Target servo position from left stick ----
        // Stick up = smaller position, Stick down = larger position
        targetServoPos -= gamepad1.left_stick_y * 0.01;
        targetServoPos = Math.max(0.0, Math.min(1.0, targetServoPos));

        // ---- Smooth transition toward target ----
        double step = 0.02;  // how fast it eases each loop (lower = smoother)
        if (servoPos < targetServoPos) {
            servoPos = Math.min(servoPos + step, targetServoPos);
        } else if (servoPos > targetServoPos) {
            servoPos = Math.max(servoPos - step, targetServoPos);
        }
        angleServo.setPosition(servoPos);

        // ---- Adjust motor power with right stick ----
        motorPower = -gamepad1.right_stick_y;
        motorPower = Math.max(0.0, Math.min(1.0, motorPower));

        // ---- Spin motor only while right trigger is held ----
        if (gamepad1.right_trigger > 0.1) {
            launchMotor.setPower(motorPower);
        } else {
            launchMotor.setPower(0);
        }

        // ---- Telemetry ----
        telemetry.addData("Servo Pos", servoPos);
        telemetry.addData("Target Servo", targetServoPos);
        telemetry.addData("Motor Power Set", motorPower);
        telemetry.addData("Trigger Held", gamepad1.right_trigger > 0.1);
        telemetry.update();
    }
}
