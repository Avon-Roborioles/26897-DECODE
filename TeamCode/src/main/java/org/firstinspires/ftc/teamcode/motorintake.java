package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "motorintake", group = "Control")
public class motorintake extends OpMode {

    private DcMotor spinnerMotor;

    // A guessed power level to hit 350-400 RPM. This is NOT guaranteed.
    private static final double TARGET_POWER = 0.3;
    private static final String MOTOR_NAME = "intake_motor";

    private boolean isMotorRunning = false;
    private boolean aButtonWasPressed = false;

    @Override
    public void init() {
        try {
            spinnerMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

            // It's good practice to set a run mode even with setPower()
            spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Status", "Initialized using setPower()");
            telemetry.addData("Target Power", TARGET_POWER);
        } catch (Exception e) {
            telemetry.addData("FATAL ERROR", "Motor not found.");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aButtonIsPressed = gamepad1.a;

        if (aButtonIsPressed && !aButtonWasPressed) {
            isMotorRunning = !isMotorRunning;

            if (isMotorRunning) {
                // RUN: Apply the constant power
                spinnerMotor.setPower(TARGET_POWER);
            } else {
                // STOP: Set power to zero
                spinnerMotor.setPower(0);
            }
        }

        // Update the state for the next loop cycle
        aButtonWasPressed = aButtonIsPressed;

        // Telemetry
        telemetry.addData("Motor State", isMotorRunning ? "RUNNING (Power: " + TARGET_POWER + ")" : "STOPPED");
        telemetry.update();
    }
}
