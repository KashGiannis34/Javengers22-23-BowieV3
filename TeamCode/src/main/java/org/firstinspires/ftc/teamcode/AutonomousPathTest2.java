package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Auto Path Test 2")
public class AutonomousPathTest2 extends LinearOpMode {
    Brobot robot;
    ColorSensor color;
    DcMotor slide;
    public DistanceSensor rightDist, leftDist;
    public Servo claw, slideServo;
    ElapsedTime etime = new ElapsedTime();
    final double rpm = 1150/28.0;
    final double ppr = 145.1*28;
    int cZeroPos, zeroPos;
    @Override
    public void runOpMode()
    {
        cZeroPos = 0;
        robot = new Brobot(hardwareMap);
        slide = hardwareMap.dcMotor.get("armMotor");
        claw = hardwareMap.servo.get("clawServo");
        slideServo = hardwareMap.servo.get("slideServo");

        Pose2d startPose = new Pose2d(40,-66,Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(58, -54), Math.toRadians(90))
                .splineTo(new Vector2d(58, -16), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(46.25,-14, Math.toRadians(-90)))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    raiseHeight(1250);
                })
                .UNSTABLE_addTemporalMarkerOffset(3,() -> {
                    slideServo.setPosition(0.54);
                })
                .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                    claw.setPosition(0.2);
                })
                .build();
        TrajectorySequence trajSeq2= robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToLinearHeading(new Pose2d(54,-14.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    slideServo.setPosition(0.1);
                    raiseHeight(440);
                })
                .build();
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(45,-14, Math.toRadians(-90)))
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(54,-14.5, Math.toRadians(0)))
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(45,-14, Math.toRadians(-90)))


                //////////////////////////////////////////////////////////
                //.splineTo(new Vector2d(60, -54), Math.toRadians(90))
                //.splineTo(new Vector2d(60,-12), Math.toRadians(90))
                //.turn(Math.toRadians(-90))

//        TrajectorySequence turnToJunction = robot.trajectorySequenceBuilder(trajSeq.end())
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(45,-14, Math.toRadians(-90)))
//                .build();
//
//        TrajectorySequence turnBacktoStack = robot.trajectorySequenceBuilder(turnToJunction.end())
//                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(54,-14.5, Math.toRadians(0)))
//                .build();

        rightDist = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDistance");
        color = hardwareMap.get(ColorSensor.class, "color");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        zeroPos = slide.getCurrentPosition();
        claw.setPosition(1);
        slideServo.setPosition(0.1);

        telemetry.addData("czeropos: ", cZeroPos);
        telemetry.update();

        waitForStart();
        etime.reset();
        raiseHeight(20);
        robot.followTrajectorySequence(trajSeq);
//        robot.followTrajectorySequence(trajSeq2);
//        for (int n = 0; n < 4; n++) {
//            robot.followTrajectorySequence(turnBacktoStack);
//            robot.followTrajectorySequence(turnToJunction);
//        }
//        sleep(100);
////        raiseHeight(1250);
//        robot.followTrajectorySequence(turnToJunction);
////        sleep(700);
////        slideServo.setPosition(0.3);
////        sleep(700);
////        claw.setPosition(0.2);
//        sleep(700);
//        robot.followTrajectorySequence(turnBacktoStack);
//        sleep(700);
//
//        int[] stackHeights = {440, 333, 220, 90, 10};
//        for (int n = 0; n < stackHeights.length; n++) {
//            raiseHeight(stackHeights[n]);
//            sleep(500);
////            slideServo.setPosition(0.7);
////            sleep(300);
////            claw.setPosition(1);
////            sleep(700);
////            raiseHeight(1250);
//            robot.followTrajectorySequence(turnToJunction);
////            slideServo.setPosition(0.3);
////            sleep(1400);
////            claw.setPosition(0.2);
//            sleep(700);
//            robot.followTrajectorySequence(turnBacktoStack);
//            sleep(700);
//        }

        Pose2d poseEstimate = robot.getPoseEstimate();
        while (opModeIsActive()) {
            telemetry.addData("red: ", color.red());
            telemetry.addData("green: ", color.green());
            telemetry.addData("blue: ", color.blue());
            telemetry.addData("Left Distance (cm): ", leftDist.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance (cm): ", rightDist.getDistance(DistanceUnit.CM));
            telemetry.addData("time elapsed: ", etime.seconds());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) ;
    }

    void raiseHeight(int num)
    {
        slide.setTargetPosition(num+zeroPos);
        if (slide.getCurrentPosition() > num+zeroPos)
            slide.setPower(-0.4);
        else
            slide.setPower(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slide.isBusy()) {

        }

        // slide.setPower(0.2);

        //clawSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
