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
                        drive.trajectorySequenceBuilder(new Pose2d(36,-66,Math.toRadians(90)))
                                .splineTo(new Vector2d(60, -54), Math.toRadians(90))
                                .splineTo(new Vector2d(60, -22), Math.toRadians(90))
                                .splineTo(new Vector2d(46,-14), Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(54,-14.5, Math.toRadians(0)))
//                                .splineTo(new Vector2d(60, -54), Math.toRadians(90))
//                                .splineTo(new Vector2d(60,-12), Math.toRadians(90))
//                                .turn(Math.toRadians(-90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}