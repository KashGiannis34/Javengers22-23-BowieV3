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
                        drive.trajectorySequenceBuilder(new Pose2d(40,-66,Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(12, -58), Math.toRadians(90))
                                .splineTo(new Vector2d(12, -18), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(24.5,-13), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(24.5, -12.00))
                                .lineToSplineHeading(new Pose2d(59,-12.00, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(46.7,-17, Math.toRadians(-90)))
                                .build()
                );

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(32.281655, 30, Math.toRadians(150), Math.toRadians(150), 13.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-30,-66,Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-12, -58), Math.toRadians(90))
                                .splineTo(new Vector2d(-12, -18), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-24.5,-13), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(-24.5, -12.00))
                                .lineToSplineHeading(new Pose2d(-59,-12.00, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-46.7,-17, Math.toRadians(270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }
}