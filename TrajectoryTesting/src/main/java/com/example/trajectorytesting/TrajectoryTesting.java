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
                        drive.trajectorySequenceBuilder(new Pose2d(32,-62,Math.toRadians(90)))
//                                .splineTo(new Vector2d(32,-44), Math.toRadians(90))
//                                .splineTo(new Vector2d(42,-11), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(32+(18/5.1),-44, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(42,-11, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}