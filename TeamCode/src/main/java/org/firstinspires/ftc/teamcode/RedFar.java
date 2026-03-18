package org.firstinspires.ftc.teamcode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.BlueShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@Autonomous
public class RedFar extends NextFTCOpMode {
    Command path1Command;

    public RedFar() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );

    }


    @Override
    public void onInit() {

        follower().setPose(new Pose(0, 0, Math.PI / 2));
        follower().setMaxPower(0.70);
        Path path1 = new Path(
                new BezierLine(
                        follower().getPose(),
                        new Pose(25, 0, Math.toRadians(90))

                )

        );
        //(3*Math.PI)/4
        path1.setLinearHeadingInterpolation(Math.PI / 2, Math.PI / 2);


        Path path2 = new Path(
                new BezierLine(
                        new Pose(0, -42.5, Math.toRadians(90)),
                        new Pose(-10,-47,Math.toRadians(135))
                )
        );
        path2.setLinearHeadingInterpolation(Math.PI / 2, (3*Math.PI) / 4);

        Path path3 =new Path(
                new BezierLine(
                        new Pose(-10,-40,Math.toRadians(135)),
                        new Pose(-27,-17,Math.toRadians(135))
                )
        );
        path3.setLinearHeadingInterpolation((3*Math.PI) / 4, (3*Math.PI) / 4);

        Path path4 = new Path(
                new BezierLine(
                        new Pose(-27,-20,Math.toRadians(135)),
                        new Pose(0, -42.5, Math.toRadians(135))
                )
        );
        path4.setLinearHeadingInterpolation((3*Math.PI) / 4, (3*Math.PI) / 4);

        Path path5 = new Path(
                new BezierLine(
                        new Pose(0, -42.5, Math.toRadians(135)),
                        new Pose(-27, -17, Math.toRadians(135))
                )
        );
        path5.setLinearHeadingInterpolation((3*Math.PI) / 4, (3*Math.PI) / 4);

        Path path6 = new Path(
                new BezierLine(
                        new Pose(-31, -50, Math.toRadians(135)),
                        new Pose(-34, -51, Math.toRadians(135))
                )
        );
        path6.setLinearHeadingInterpolation((3*Math.PI) / 4, (3*Math.PI) / 4);

        Path path7 = new Path(
                new BezierLine(
                        new Pose(-34, -51, Math.toRadians(135)),
                        new Pose(-0, -42.5, Math.toRadians(135))
                )
        );
        path7.setLinearHeadingInterpolation((3*Math.PI) / 4, (3*Math.PI) / 4);

        path1Command = new FollowPath(
                path1
        );

    }


    @Override
    public void onStartButtonPressed(){
        new SequentialGroup(
                path1Command
        ).schedule();
    }

}