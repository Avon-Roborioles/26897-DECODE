package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp
public class IntakeTest extends LinearOpMode {
    private DcMotor motor;
    private CRServo crservo;
    private CRServo crservo1;
    private CRServo crservo2;
    private int targetDistance = 0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "intakemotor");
        crservo = hardwareMap.get(CRServo.class, "intakeservo");
        crservo1 = hardwareMap.get(CRServo.class, "intakeservo1");
        crservo2 = hardwareMap.get(CRServo.class, "intakeservo2");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Change target with buttons
            if (gamepad1.a) targetDistance = 520;
            if (gamepad1.b) targetDistance = 0;

            motor.setTargetPosition(targetDistance);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.5);

            // Servo control
            if (gamepad1.right_trigger > 0.001) {
                crservo.setPower(1);
                crservo1.setPower(-1);
                crservo2.setPower(-1);
            } else {
                crservo.setPower(0);
                crservo1.setPower(0);
                crservo2.setPower(0);
            }

            telemetry.addData("Target Position", targetDistance);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Busy", motor.isBusy());
            telemetry.update();
        }

        motor.setPower(0);
    }
}
