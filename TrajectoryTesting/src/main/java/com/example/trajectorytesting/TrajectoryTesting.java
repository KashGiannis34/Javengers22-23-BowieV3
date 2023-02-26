package com.example.trajectorytesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        double turnY = -44;
        Pose2d startPose = new Pose2d(32,-62,Math.toRadians(90));
        Pose2d endPose = new Pose2d(37,-11,Math.toRadians(0));
        double slope = ((double)(endPose.getY()-startPose.getY()))/(endPose.getX()-startPose.getX());
        Pose2d middlePose = new Pose2d(startPose.getX()+18/slope, turnY);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(32.281655, 30, Math.toRadians(150), Math.toRadians(150), 13.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(middlePose)
                                .lineToSplineHeading(endPose)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.3f)
                .addEntity(myBot)
                .start();

    }
}