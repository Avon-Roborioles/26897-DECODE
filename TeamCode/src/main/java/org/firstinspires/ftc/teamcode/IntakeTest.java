package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeTest extends LinearOpMode {
    private DcMotor intakeMotor;
    private Servo intakeServo;
    private double servopos;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intakeMotor.setPower(1);
            } else if (gamepad1.b) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            while(gamepad1.dpad_up) {
                this.servopos += 0.05;
                intakeServo.setPosition(servopos);
            }
            while(gamepad1.dpad_down) {
                this.servopos -= 0.05;
                intakeServo.setPosition(servopos);
            }
        }
    }
}
