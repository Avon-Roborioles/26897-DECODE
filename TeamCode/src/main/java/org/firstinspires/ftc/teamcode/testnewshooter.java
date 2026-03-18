package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo; // Changed from CRServo
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(name = "Test Teleop")
public class testnewshooter extends LinearOpMode {
    private DcMotorEx shooter1, shooter2;
    double manualspeed = 800;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class,"shooter2");

        shooter1.setVelocityPIDFCoefficients(500,0,11.2,17);
        shooter2.setVelocityPIDFCoefficients(500,0,11.2,17);



        telemetry.addLine("right bumper -- more speed");
        telemetry.addLine("left bumper -- less speed");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_bumper) {
                manualspeed += 25;
            }
            if(gamepad1.left_bumper) {
                manualspeed -= 25;
            }
            shooter1.setVelocity(manualspeed);
            shooter2.setVelocity(-manualspeed);
            telemetry.addData("Current Velocity", manualspeed);
            telemetry.update();
        }
    }
}