package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous (name = "Auto Test")
public class AutoTest extends LinearOpMode {
    Brobot robot;

    @Override
    public void runOpMode()
    {
        robot = new Brobot(hardwareMap);

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10,10),0)
                .waitSeconds(1)
                .strafeRight(5)
                .forward(5)
                .build();

        waitForStart();

        robot.followTrajectorySequence(trajSeq);
    }
}
