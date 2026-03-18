//package org.firstinspires.ftc.teamcode;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//@TeleOp
//@Configurable
//public class PIDFShooterTune extends OpMode {
//    private DcMotorEx shooter;
//    public static int speed = 700;
//    public static double p = 0;
//    public static double i = 0;
//    public static double d = 0;
//    public static double f = 0;
//    PIDFCoefficients pidfCoefficients = new PIDFCoefficients();
//    @Override
//    public void init() {
//        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    @Override
//    public void loop() {
//        if(gamepad1.a) {
//            p = p + 0.05;
//        }
//        if(gamepad1.b) {
//            i = i + 0.05;
//        }
//        if(gamepad1.y) {
//            d = d + 0.05;
//        }
//        if(gamepad1.x) {
//            f = f + 0.05;
//        }
//        pidfCoefficients.p=p;
//
//        shooter.setVelocityPIDFCoefficients(p,i,d,f);
//        shooter.setVelocity(speed);
//        shooter.setPower(1);
//        PanelsTelemetry.INSTANCE.getTelemetry().addData("Current Velocity",shooter.getVelocity());
//
//        PanelsTelemetry.INSTANCE.getTelemetry().update();
//    }
//}
