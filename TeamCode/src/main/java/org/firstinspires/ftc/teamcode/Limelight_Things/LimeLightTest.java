//package org.firstinspires.ftc.teamcode.Vision;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//
//
//public class LimelightTest {
//
//    private Limelight3A limelight;
//    private IMU imu;
//    private Pose pose;
//    private double startHeading;
//
//
//    public LimelightTest(HardwareMap hardwareMap){
//        limelight = hardwareMap.get(Limelight3A.class,"Limelight");
//        imu = hardwareMap.get(IMU.class,"imu");
//        limelight.pipelineSwitch(0);
//        limelight.start();
//        pose = new Pose();
//
//        startHeading = imu.getRobotYawPitchRollAngles().getYaw();
//    }
//
//    public void run(){
//        LLResult result = limelight.getLatestResult();
//        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw()-startHeading;
//        limelight.updateRobotOrientation(robotYaw);
//
//
//        pose.setHeading(robotYaw);
//        Pose3D pose3D = result.getBotpose_MT2();
//        pose.setX(pose3D.getPosition().x);
//        pose.setY(pose3D.getPosition().y);
//    }
//    public Pose getCurPose(){
//        return pose;
//    }
//}