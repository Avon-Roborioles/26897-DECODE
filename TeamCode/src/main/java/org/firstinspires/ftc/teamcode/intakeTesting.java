package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class intakeTesting extends LinearOpMode {
    private DcMotor intakeMotor;
    private Servo intakeServo;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        waitForStart();

        while (opModeIsActive()) {
            while(gamepad1.right_trigger > 0.01) {
                intakeMotor.setPower(1);
            }
            if(gamepad1.a) {
                intakeServo.setPosition(0.467);
            }
        }
    }
}
