package org.firstinspires.ftc.teamcode.Kevin_Stuff;

import static java.lang.Math.PI;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryManager;

import java.util.function.Supplier;
@Disabled

@TeleOp
public class TeleOpKevin extends OpMode {
    private Follower follower;
    private static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryManager;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0,PI));
        follower.update();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierCurve(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        follower.setTeleOpDrive(gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_stick_x, false);
    }
}
