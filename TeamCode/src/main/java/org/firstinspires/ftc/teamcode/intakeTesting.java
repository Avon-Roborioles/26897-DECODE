package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

@TeleOp
public class intakeTesting extends LinearOpMode {
    private MecanumDrivetrain driveTrain;
    private DcMotor intakeMotor;
    private Servo intakeServo;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        //intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        driveTrain = new MecanumDrivetrain(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y; // Corrected variable name for clarity
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            driveTrain.drive(strafe, forward, turn);


            if(gamepad1.right_trigger > 0.67) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }


            telemetry.addData("turn Value: ", turn);
            telemetry.update();
//            if(gamepad1.a) {
//                intakeServo.setPosition(0.467);
//            }
        }
    }
}
