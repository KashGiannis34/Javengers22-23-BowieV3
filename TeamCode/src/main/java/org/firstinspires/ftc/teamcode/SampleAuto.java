/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.barcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Sample Auto")
public class SampleAuto extends LinearOpMode
{
    // cone alignment variables...
    int width = 800;
    int height = 448;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Brobot robot;
    ColorSensor color;
    DcMotor carousel, slide;
    public DistanceSensor rightDist, leftDist;
    public Servo claw, slideServo;
    ElapsedTime etime = new ElapsedTime();
    final double rpm = 1150/28.0;
    final double ppr = 145.1*28;
    int cZeroPos;



    DcMotor arm, leftFront, leftBack, rightFront, rightBack;
    int zeroPos = arm.getCurrentPosition();

    Servo clawServo;
    Servo armservo;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    Pose2d startPose = new Pose2d(0, 0);

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Right"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            sleep(20);
            telemetry.update();
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        etime.reset();

        /* Actually do something useful */

        //        robot.followTrajectorySequence(trajSeq);
        sleep(100);
        raiseHeight(1250);
        sleep(700);
        setAngle(112);
//        sleep(700);
//        slideServo.setPosition(0.7);
//        sleep(700);
//        claw.setPosition(0.15);
//        sleep(700);
//        setAngle(0);
//        sleep(700);
//
//        int[] stackHeights = {440, 333, 225, 118, 10};
//        for (int n = 0; n < 1/*stackHeights.length*/; n++) {
//            raiseHeight(stackHeights[n]);
//            sleep(1400);
//            slideServo.setPosition(0.81);
//            sleep(300);
//            claw.setPosition(1);
//            sleep(700);
//            raiseHeight(1250);
//            sleep(700);
//            setAngle(112);
//            slideServo.setPosition(0.7);
//            sleep(700);
//            claw.setPosition(0.2);
//            sleep(700);
//        }
//        setAngle(0);
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


        if(tagOfInterest == null || tagOfInterest.id == LEFT)
        {
            // LEFT code
            leftPark();
        }
        else if (tagOfInterest.id == MIDDLE)
        {
            // MIDDLE code
            middlePark();
        }
        else
        {
            // RIGHT code
            rightPark();
        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void leftPark()
    {

    }

    void middlePark()
    {

    }

    void rightPark()
    {

    }

    void setAngle(int angle)
    {
        carousel.setTargetPosition(cZeroPos+(int)(ppr*angle/360.0));
        if (carousel.getCurrentPosition() > cZeroPos+(int)(ppr*angle/360.0))
            carousel.setPower(-1);
        else
            carousel.setPower(1);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (carousel.isBusy()) {

        }
        carousel.setPower(0);
    }

    void raiseHeight(int num)
    {
        slide.setTargetPosition(num+zeroPos);
        if (slide.getCurrentPosition() > num+zeroPos)
            slide.setPower(-0.3);
        else
            slide.setPower(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slide.isBusy()) {

        }

        slide.setPower(0.2);

        //clawSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void sleepSec(int seconds)
    {
        try {
            TimeUnit.SECONDS.sleep(seconds);
        }
        catch (InterruptedException e)
        {
            System.out.println("interrupted");
        }
    }
}