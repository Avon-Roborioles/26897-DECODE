//package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
//public class IntakeTest extends LinearOpMode {
//  private DcMotor intakeMotor;
//private Servo intakeServo;
//private double servopos;

        waitForStart();
        servo.setPosition(servoPosition);

        while (opModeIsActive()) {
            if(gamepad1.aWasReleased()) targetVelocity += 50;
            if(gamepad1.bWasReleased()) targetVelocity -= 50;
            motor.setVelocity(targetVelocity);

            if(gamepad1.xWasReleased()) servoPosition += 0.0075;
            if(gamepad1.yWasReleased()) servoPosition -= 0.0075;
            servo.setPosition(servoPosition);

            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", motor.getVelocity());
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
        motor.setVelocity(0);
    }
}
