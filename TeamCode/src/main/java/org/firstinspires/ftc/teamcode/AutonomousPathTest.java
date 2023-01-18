package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Auto Path Test")
public class AutonomousPathTest extends LinearOpMode {
    Brobot robot;
    ColorSensor color;
    DcMotor carousel, slide;
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
        carousel = hardwareMap.dcMotor.get("turnTable");
        claw = hardwareMap.servo.get("clawServo");
        slideServo = hardwareMap.servo.get("slideServo");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Pose2d startPose = new Pose2d(36,-66,Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(60, -54), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(54,-13, Math.toRadians(0)), Math.toRadians(90))
                //////////////////////////////////////////////////////////
                //.splineTo(new Vector2d(60, -54), Math.toRadians(90))
                //.splineTo(new Vector2d(60,-12), Math.toRadians(90))
                //.turn(Math.toRadians(-90))
                .build();

        rightDist = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDistance");
        color = hardwareMap.get(ColorSensor.class, "color");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cZeroPos = carousel.getCurrentPosition();
//        zeroPos = slide.getCurrentPosition();
        claw.setPosition(1);

        telemetry.addData("czeropos: ", cZeroPos);
        telemetry.update();

        waitForStart();
        etime.reset();

//        robot.followTrajectorySequence(trajSeq);
        sleep(100);
        raiseHeight(1250);
        setAngle(1263);
        sleep(700);
        slideServo.setPosition(0.67);
        sleep(700);
        claw.setPosition(0.2);
        sleep(700);
        setAngle(0);
        sleep(700);

        int[] stackHeights = {440, 333, 220, 90, 10};
        for (int n = 0; n < stackHeights.length; n++) {
            raiseHeight(stackHeights[n]);
            sleep(500);
            slideServo.setPosition(0.7);
            sleep(300);
            claw.setPosition(1);
            sleep(700);
            raiseHeight(1250);
            setAngle(1263);
            slideServo.setPosition(0.67);
            sleep(1400);
            claw.setPosition(0.2);
            sleep(700);
            setAngle(0);
            sleep(700);
        }
        setAngle(0);
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

    void setAngle(int pos)
    {
        carousel.setTargetPosition(pos);
        if (carousel.getCurrentPosition() > pos)
            carousel.setPower(-0.5);
        else
            carousel.setPower(0.5);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //sleep(100);
        while (carousel.isBusy()) {

        }
        //carousel.setPower(0);
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
