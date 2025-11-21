package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.limelightcommand;
import org.firstinspires.ftc.teamcode.limelightsubsystem;

import java.util.List;

@TeleOp

public class limelighttracker extends LinearOpMode {
    private MecanumDrivetrain driveTrain;
    private Limelight3A limelight;
    private Servo servo;
    private double servoPos = 0.5;
    public static int tag;

    private limelightcommand limelightCommand;
    private limelightsubsystem limelightSubsystem;
    private LLResult result;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware
        servo = hardwareMap.get(Servo.class, "pan_servo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        driveTrain = new MecanumDrivetrain(hardwareMap);


        limelightCommand = new limelightcommand(limelightSubsystem, result);

        telemetry.setMsTransmissionInterval(11);
        servo.setPosition(0.67);
        limelight.pipelineSwitch(0);
        limelight.start();



        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y; // Corrected variable name for clarity
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            driveTrain.drive(strafe, forward, turn);


            LLStatus status = limelight.getStatus();

            // ---- Limelight logic (servo adjustment) ----
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                //telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("tags",result.getFiducialResults());
                telemetry.addData("distance", getDistance());
//                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                        status.getTemp(), status.getCpu(),(int)status.getFps());
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                    if (fr.getFiducialId() == 21) {
//                        tag = 21;
//                        limelight.pipelineSwitch(3);
//                    } else if (fr.getFiducialId() == 22) {
//                        tag = 22;
//                        limelight.pipelineSwitch(3);
//                    } else if (fr.getFiducialId() == 23) {
//                        tag = 23;
//                        limelight.pipelineSwitch(3);
//                    }
//                }
//                telemetry.addData("tag", tag);
                telemetry.update();



                if (result.getTx() != 0) {
                    servoPos -= 0.0001 * result.getTx();
                }
            }

            if (servoPos > 0.95) {
                servoPos = 0.95;
            } else if (servoPos < 0.15) {
                servoPos = 0.15;
            }
            servo.setPosition(servoPos);
        }
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();


    }

    int getTag(){
        return tag;
    }

    double getDistance() {
        LLResult result = limelight.getLatestResult();
        double targetOffsetAngle_Vertical = result.getTy();

        double limelightMountAngleDegrees = 0;
        double limelightLensHeightInches = 14.9;
        double goalHeightInches = 28.5;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    double getYawAprilTag() {
        LLResult result = limelight.getLatestResult();
        return result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
    }
}