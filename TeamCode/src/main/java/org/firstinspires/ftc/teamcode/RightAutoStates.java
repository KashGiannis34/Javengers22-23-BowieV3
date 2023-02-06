package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.barcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Right Auto States")
public class RightAutoStates extends LinearOpMode {
    Brobot robot;
    //ColorSensor color;
    DcMotor slide, carousel;
    public DistanceSensor rightDist, leftDist;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public Servo claw, slideServo;
    ElapsedTime etime = new ElapsedTime();
    final double rpm = 1150/28.0;
    final double ppr = 1992.6;
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
        carousel = hardwareMap.dcMotor.get("turnTable");
        claw = hardwareMap.servo.get("clawServo");
        slideServo = hardwareMap.servo.get("slideServo");

        Pose2d startPose = new Pose2d(40,-66,Math.toRadians(90));
        robot.setPoseEstimate(startPose);


        rightDist = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDistance");
        //color = hardwareMap.get(ColorSensor.class, "color");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        zeroPos = slide.getCurrentPosition();
        claw.setPosition(1);
        slideServo.setPosition(0.1);
        sleep(10);
        raiseHeight(20);
        sleep(10);
        setAngle(0);
        sleep(10);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Right"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
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

        sleep(100);
        raiseHeight(1100);
        setAngle(125);
        slideServo.setPosition(0.38);
        sleep(500);
        claw.setPosition(0.4);
        sleep(150);
        setAngle(0);
        sleep(500);

        int[] stackHeights = {360, 273, 185, 98, 10};
        for (int n = 0; n < stackHeights.length; n++) {
            raiseHeight(stackHeights[n]);
            claw.setPosition(1);
            sleep(300);
            raiseHeight(1100);
            setAngle(125);
            slideServo.setPosition(0.38);
            sleep(500);
            claw.setPosition(0.4);
            sleep(150);
            setAngle(0);
        }
        setAngle(0);
        sleep(50);
        raiseHeight(10);
        sleep(100);
        slideServo.setPosition(0.1);
        sleep(50);


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
        while (opModeIsActive()) {
            if (slide.getCurrentPosition() > num) {
                slide.setTargetPosition(num);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-0.7);
            }
            else {
                slide.setTargetPosition(num);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            }

            if (Math.abs(slide.getCurrentPosition()-num) <= 10) {
                slide.setPower(0.2);
                break;
            }
        }
    }

    void setAngle(int angle)
    {
        int pos = (int)(ppr*angle/360.0);
        while (opModeIsActive()) {
            if (carousel.getCurrentPosition() > (int) (ppr * angle / 360.0)) {
                carousel.setTargetPosition(pos);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(-0.7);
            }
            else {
                carousel.setTargetPosition(pos);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(0.7);
            }

            if (Math.abs(carousel.getCurrentPosition()-pos) <= 3) {
                carousel.setPower(0);
                break;
            }
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
}
