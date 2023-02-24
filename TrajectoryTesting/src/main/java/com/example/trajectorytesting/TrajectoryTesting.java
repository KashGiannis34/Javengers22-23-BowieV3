package com.example.trajectorytesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(32.281655, 30, Math.toRadians(150), Math.toRadians(150), 13.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40,-62,Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-12, -55), Math.toRadians(90))
                                .splineTo(new Vector2d(-12, -25), Math.toRadians(90))
                                .splineTo(new Vector2d(-47.5,-8), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}