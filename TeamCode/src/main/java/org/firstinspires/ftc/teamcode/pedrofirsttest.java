package org.firstinspires.ftc.teamcode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous
public class pedrofirsttest extends NextFTCOpMode {
    Command path1Command;
    Command path2Command,path3Command;

    public pedrofirsttest(){
        addComponents(
                new PedroComponent(Constants::createFollower)
        );

    }

    @Override
    public void onInit(){
        follower().setPose(new Pose(0,0,3*Math.PI/2));
        Path path1 = new Path(
                new BezierLine(
                        follower().getPose(),
                        new Pose(-19,-50.399,Math.toRadians(-180))
                )
        );
        path1.setLinearHeadingInterpolation(3*Math.PI/2,Math.PI);
        Path path2 = new Path(
                new BezierLine(
                        new Pose(-19,-50.399,Math.toRadians(-180)),
                        new Pose(-39,-50.399,Math.toRadians(-180))
                )
        );
        Path path3 =new Path(
                new BezierLine(
                        new Pose(-39,-50.399,Math.toRadians(-180)),
                        new Pose(-8.01,-49.54,Math.toRadians(-151.19))
                )
        );
        path3.setLinearHeadingInterpolation(Math.toRadians(-180),Math.toRadians(-151.19));
        path1Command = new FollowPath(
                path1
        );
        path2Command = new FollowPath(
                path2
        );
        path3Command = new FollowPath(
                path3
        );

    }

    @Override
    public void onStartButtonPressed(){
        new SequentialGroup(
                path1Command,
                path2Command,
                path3Command
        ).schedule();
    }

}