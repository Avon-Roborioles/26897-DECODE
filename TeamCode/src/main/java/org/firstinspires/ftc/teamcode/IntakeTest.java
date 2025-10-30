package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp
public class IntakeTest extends LinearOpMode {
    private DcMotorEx motor;
    private CRServo crservo;
    private CRServo crservo1;
    private CRServo crservo2;
    private int targetDistance = 50;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "intakemotor");
        crservo = hardwareMap.get(CRServo.class, "intakeservo");
        crservo1 = hardwareMap.get(CRServo.class, "intakeservo1");
        crservo2 = hardwareMap.get(CRServo.class, "intakeservo2");

        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        crservo.setPower(0);

        while (opModeIsActive()) {
            if(gamepad1.aWasReleased()) {targetDistance = 50;}
            if(gamepad1.bWasReleased()) {targetDistance = 0;}
            motor.setTargetPosition(targetDistance);

            while (gamepad1.right_trigger > 0.001) {
                crservo.setPower(1);
                crservo1.setPower(1);
                crservo2.setPower(1);
            }
//            if(gamepad1.rightBumperWasReleased()) {crservo1.setPower(1);}
//            if(gamepad1.leftBumperWasReleased()) {crservo1.setPower(0);}
//            if(gamepad1.rightBumperWasReleased()) {crservo2.setPower(1);}
//            if(gamepad1.leftBumperWasReleased()) {crservo2.setPower(0);}

            telemetry.addData("Target Position", targetDistance);
            telemetry.addData("Current Velocity", motor.getVelocity());
            telemetry.addData("Servo Power", crservo.getPower());
            telemetry.update();
        }
        motor.setVelocity(0);
    }
}
