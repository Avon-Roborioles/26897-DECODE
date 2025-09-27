package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Trigger Motor Control", group="Linear Opmode")
public class testLaunch extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {
        // Map motor (make sure "motorName" matches the config in the RC app)
        motor = hardwareMap.get(DcMotor.class, "bomboclatLauncher");

        // Set motor direction if needed
        motor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            // Right trigger value ranges from 0.0 to 1.0
            double triggerPower = gamepad1.right_trigger;

            // Spin motor only while trigger is pressed
            if (triggerPower > 0.1) { // small threshold to avoid noise
                motor.setPower(1); // scales speed with how much you press
            } else {
                motor.setPower(0);
            }

            telemetry.addData("Trigger", triggerPower);
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}
