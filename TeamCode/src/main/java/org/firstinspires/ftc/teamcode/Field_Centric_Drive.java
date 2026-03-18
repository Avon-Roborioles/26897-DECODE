//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.Subsystems.FieldDrivetrain;
//import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
//
//import java.util.List;
//
//@TeleOp
//public class Field_Centric_Drive extends LinearOpMode {
//    private FieldDrivetrain driveTrain;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        driveTrain = new FieldDrivetrain(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            driveTrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
//            if (gamepad1.y) {
//                driveTrain.resetHeading();
//                gamepad1.rumble(20000);
//            }
//
//        }
//    }
//}
