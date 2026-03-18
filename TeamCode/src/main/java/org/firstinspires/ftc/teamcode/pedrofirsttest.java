package org.firstinspires.ftc.teamcode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@Autonomous
public class pedrofirsttest extends NextFTCOpMode {

    private ShooterSubsystem shooter;
    Command shotsecond;
    Command shotthird;
    Command path1Command;
    Command path2Command, path3Command;
    private DcMotor intake;
    private DcMotor kicker;
    private Servo artifactServo;
    private ArtifactSensor artifactSensor;

    private final double[] SLOT_POSITIONS = {0.2256, 0.3023, 0.3828};
    private int currentSlotIndex = 0;
    private final double SERVO_MOVE_DELAY = 150;
    private ElapsedTime timer = new ElapsedTime();

    private ArtifactColor targetColor = ArtifactColor.NOTHING;


    // Indexing State Variables
    private boolean intakeSeekingEmpty = false;
    private int intakeSlotsChecked = 0;
    private boolean magazineFull = false;
    private double sensorCheckTime = -1;

    public pedrofirsttest() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );

    }


    @Override
    public void onInit() {
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotor.class, "intake");
        kicker = hardwareMap.get(DcMotor.class, "kicker");
        artifactServo = hardwareMap.get(Servo.class, "sorterservo");
        artifactServo.setPosition(SLOT_POSITIONS[0]);
        artifactSensor = new ArtifactSensor(hardwareMap);

        follower().setPose(new Pose(0, 0, Math.PI / 2));
        follower().setMaxPower(0.75);
        Path path1 = new Path(
                new BezierLine(
                        follower().getPose(),
                        new Pose(0, -42.5, Math.toRadians(90))

                )

        );
        //(3*Math.PI)/4
        path1.setLinearHeadingInterpolation(Math.PI / 2, Math.PI / 2);


        Path path2 = new Path(
                new BezierLine(
                        new Pose(0, -42.5, Math.toRadians(90)),
                        new Pose(13,-35,Math.toRadians(90))
                )
        );
        path2.setLinearHeadingInterpolation(Math.PI / 2, Math.PI / 4);

        Path path3 =new Path(
                new BezierLine(
                        new Pose(13,-35,Math.toRadians(-180)),
                        new Pose(35,-12,Math.toRadians(-151.19))
                )
        );
//        path3.setLinearHeadingInterpolation(Math.toRadians(-180),Math.toRadians(-151.19));
        path1Command = new FollowPath(
                path1
        );

        path3Command = new ParallelGroup(
                intakefr(),
                new FollowPath(path3)
        );


        path2Command = new FollowPath(
                path2
        );

    }

    public Command intakefr() {
        return new Command() {
            boolean intakeRunning;
            ElapsedTime runTimer = new ElapsedTime();

            @Override
            public void start() {
                runTimer.reset();
                intakeRunning = true;
            }

            @Override
            public void update() {
                if (!intakeRunning) magazineFull = false;

                // A. Start searching for empty slot if we just picked something up
                if (intakeRunning && !intakeSeekingEmpty && !magazineFull) {
                    if (artifactSensor.read() != ArtifactColor.NOTHING) {
                        intakeSeekingEmpty = true;
                        intakeSlotsChecked = 0;
                        incrementSlot();
                        sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
                    }
                }

                // B. Handle seeking the next empty slot
                if (intakeSeekingEmpty) {
                    if (timer.milliseconds() >= sensorCheckTime) {
                        if (artifactSensor.read() == ArtifactColor.NOTHING) {
                            intakeSeekingEmpty = false;
                        } else {
                            intakeSlotsChecked++;
                            if (intakeSlotsChecked >= 3) {
                                intakeSeekingEmpty = false;
                                magazineFull = true;
                            } else {
                                incrementSlot();
                                sensorCheckTime = timer.milliseconds() + SERVO_MOVE_DELAY;
                            }
                        }
                    }
                }

                // Apply power logic
                if (magazineFull) intake.setPower(0);
                else intake.setPower(1);
            }

            @Override
            public void stop(boolean interrupted) {
                intake.setPower(0);
                intakeRunning = false;
            }

            @Override
            public boolean isDone(){
                return runTimer.seconds() >= 9;
            }
        };
    }

    public Command spinKickOnce() {
        return new Command() {
            ElapsedTime actionTimer = new ElapsedTime();
            boolean finished = false;

            @Override
            public void start() {
                actionTimer.reset();
                finished = false;
            }

            @Override
            public void update() {
                double elapsed = actionTimer.milliseconds();

                // Sequence of events based on time instead of sleeps
                if (elapsed < 500) {
                    kicker.setPower(1);    // Kick out
                } else if (elapsed < 600) {
                    kicker.setPower(-1);   // Retract
                } else if (elapsed < 850) {
                    kicker.setPower(0);    // Wait for settle
                    intake.setPower(0);
                } else {
                    // Final actions before ending
                    incrementSlot();
                    targetColor = ArtifactColor.NOTHING;
                    finished = true;
                }
            }

            @Override
            public void stop(boolean interrupted) {
                kicker.setPower(0); // Safety
            }

            @Override
            public boolean isDone() {
                return finished;
            }
        };
    }

    private void incrementSlot() {
        currentSlotIndex++;
        if (currentSlotIndex >= SLOT_POSITIONS.length) {
            currentSlotIndex = 0;
        }
        double cur = artifactServo.getPosition();
        artifactServo.setPosition(cur + 0.0796);
    }

    @Override
    public void onStartButtonPressed(){
        shooter.updateCommand().schedule();
        new SequentialGroup(
                path1Command,
                spinKickOnce(),
                spinKickOnce(),
                spinKickOnce(),
                spinKickOnce(),
                path2Command,
                new InstantCommand(() ->{PedroComponent.follower().setMaxPower(0.20);}),
                path3Command
        ).schedule();
    }

}