package org.firstinspires.ftc.teamcode.archived;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Brobot;
import org.firstinspires.ftc.teamcode.barcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Right Auto Basic")
@Disabled
public class RightAutoBasic extends LinearOpMode {
    Brobot robot;
    //ColorSensor color;
    DcMotor slide;
    public DistanceSensor rightDist, leftDist;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public Servo claw, slideServo;
    ElapsedTime etime = new ElapsedTime();
    final double rpm = 1150/28.0;
    final double ppr = 145.1*28;
    int cZeroPos, zeroPos;
    final double extendedPos = 0.595;
    final double turnAmount = 126;

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
                .splineToConstantHeading(new Vector2d(12, -58), Math.toRadians(90))
                .splineTo(new Vector2d(12, -18), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(24.15,-13))
                .UNSTABLE_addTemporalMarkerOffset(1, () ->{
                    raiseHeight(2900);
                })
                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    slideServo.setPosition(0.48);
                })
                .UNSTABLE_addTemporalMarkerOffset(5, () ->{
                    claw.setPosition(0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(7, () ->{
                    slideServo.setPosition(0.1);
                })
                .UNSTABLE_addTemporalMarkerOffset(8, () ->{
                    slide.setPower(0);
                })
                .waitSeconds(10)
                .lineToConstantHeading(new Vector2d(24.15, -15))
                .build();

        TrajectorySequence leftPark = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(8,-16))
                .build();

        TrajectorySequence middlePark = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(36,-16))
                .build();

        TrajectorySequence rightPark = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(60,-16))
                .build();

        rightDist = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDistance");
        //color = hardwareMap.get(ColorSensor.class, "color");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        zeroPos = slide.getCurrentPosition();
        claw.setPosition(1);
        slideServo.setPosition(0.1);
        sleep(10);
        raiseHeight(20);

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

        robot.followTrajectorySequence(trajSeq);
//        robot.followTrajectorySequence(trajSeq2);

        if (tagOfInterest == null || tagOfInterest.id == LEFT)
            robot.followTrajectorySequence(leftPark);
        else if (tagOfInterest.id == MIDDLE)
            robot.followTrajectorySequence(middlePark);
        else if (tagOfInterest.id == RIGHT)
            robot.followTrajectorySequence(rightPark);
        else
            robot.followTrajectorySequence(leftPark);

        Pose2d poseEstimate = robot.getPoseEstimate();
        while (opModeIsActive()) {
            //telemetry.addData("red: ", color.red());
            //telemetry.addData("green: ", color.green());
            //telemetry.addData("blue: ", color.blue());
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
}
