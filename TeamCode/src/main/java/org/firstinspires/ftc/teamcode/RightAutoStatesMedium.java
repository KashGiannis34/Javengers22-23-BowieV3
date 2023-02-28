package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.barcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Right Auto States Medium")
public class RightAutoStatesMedium extends LinearOpMode {
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
    boolean exit = false;
    boolean exitTrajAsync = false;

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

    enum State
    {
        PARK_LEFT,
        PARK_RIGHT,
        PARK_MIDDLE
    }
    State currState;

    @Override
    public void runOpMode()
    {
        cZeroPos = 0;
        robot = new Brobot(hardwareMap);
        slide = hardwareMap.dcMotor.get("armMotor");
        carousel = hardwareMap.dcMotor.get("turnTable");
        claw = hardwareMap.servo.get("clawServo");
        slideServo = hardwareMap.servo.get("slideServo");

        rightDist = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDistance");
        //color = hardwareMap.get(ColorSensor.class, "color");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double turnY = -44;
        Pose2d startPose = new Pose2d(32,-62,Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        Pose2d endPose = new Pose2d(41.3,-7.5,Math.toRadians(0));
        double slope = ((double)(endPose.getY()-startPose.getY()))/(endPose.getX()-startPose.getX());
        Pose2d middlePose = new Pose2d(startPose.getX()+(turnY-startPose.getY())/slope, turnY);

        TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(startPose)
//                .splineToConstantHeading(new Vector2d(12, -58), Math.toRadians(90))
//                .splineTo(new Vector2d(12, -28), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(41.5,-11), Math.toRadians(0))
                .lineToSplineHeading(middlePose)
                .addTemporalMarker(0, () ->
                {
                    slide.setTargetPosition(390);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);

                    carousel.setTargetPosition(0);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(1);
                })
                .lineToSplineHeading(endPose)
                .build();

        TrajectorySequence parkRight = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(60.5, -7.5))
                .addTemporalMarker(0, () ->{
                    carousel.setTargetPosition(0);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(1);
                })
                .addTemporalMarker(0.4, () ->{
                    slide.setTargetPosition(10);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                })
                .build();

        TrajectorySequence parkMiddle = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(36.5, -7.5))
                .addTemporalMarker(0, () ->{
                    carousel.setTargetPosition(0);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(1);
                })
                .addTemporalMarker(0.4, () ->{
                    slide.setTargetPosition(10);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                })
                .build();

        TrajectorySequence parkLeft = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(14.5, -7.5))
                .addTemporalMarker(0, () ->{
                    carousel.setTargetPosition(0);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(1);
                })
                .addTemporalMarker(0.4, () ->{
                    slide.setTargetPosition(10);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                })
                .build();

        claw.setPosition(1);
        slideServo.setPosition(0.6);
        sleep(50);
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

        int max = 390;
        int min = 10;
        setMotorPos3(-29, 1170);
        slideServo.setPosition(0.87);
        sleep(600);
        claw.setPosition(0.4);
        sleep(300);
        slideServo.setPosition(0.6);
        claw.setPosition(0.4);

        robot.followTrajectorySequence(trajSeq);


        int[] stackHeights = {max, min+(max-min)*3/4+20, min+(max-min)*2/4, min+(max-min)/4, min};
        for (int n = 0; n < stackHeights.length; n++) {
//            if (etime.seconds() >= 24)
//                break;
            setMotorPosExtend(0, stackHeights[n], 0.95);
            slideServo.setPosition(1);
            if (n==0)
                sleep(300);
            else
                sleep(200);
            claw.setPosition(1);
            sleep(500);
            raiseHeightAndServoAndAngle(1985, 0.6, 145, stackHeights[n]+180);
            slideServo.setPosition(0.915);
            sleep(500);

            if (n >= stackHeights.length-2)
                claw.setPosition(0.4);
            else
                claw.setPosition(0);
            sleep(300);
            slideServo.setPosition(0.6);
            sleep(100);
        }

        claw.setPosition(0.4);
        slideServo.setPosition(0.6);
        if (tagOfInterest == null || tagOfInterest.id == RIGHT)
            robot.followTrajectorySequence(parkRight);
        else if (tagOfInterest.id == MIDDLE)
            robot.followTrajectorySequence(parkMiddle);
        else if (tagOfInterest.id == LEFT)
            robot.followTrajectorySequence(parkLeft);
        else
            robot.followTrajectorySequence(parkRight);

        while (!isStopRequested() && opModeIsActive())
        {
            telemetry.addData("slideServo pos: ", slideServo.getPosition());
            telemetry.update();
        }
    }

    void raiseHeight(double num)
    {
        while (opModeIsActive()) {
            if (slide.getCurrentPosition() > num) {
                slide.setTargetPosition((int) num);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-0.8);
            }
            else {
                slide.setTargetPosition((int) num);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            }

            if (slide.getCurrentPosition()-num >= 0) {
                slide.setPower(0.2);
                break;
            }
        }
    }

    void raiseHeightAndServoAndAngle(double num, double servoPos, double angle, int heightExtend)
    {
        int pos = (int)(ppr*angle/360.0);
        int count = 0;
        double start = 0;
        while (opModeIsActive()) {
            if (slide.getCurrentPosition() > num) {
                slide.setTargetPosition((int) num);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-1);
            }
            else if (slide.getCurrentPosition() == num)
            {
                slide.setPower(0.2);
            }
            else {
                slide.setTargetPosition((int) num);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            }

            if (slide.getCurrentPosition() >= heightExtend) {
                slideServo.setPosition(servoPos);
                if (count == 0)
                    start = System.currentTimeMillis();

                if (System.currentTimeMillis() - start >= 550) {
                    if (carousel.getCurrentPosition() > pos) {
                        carousel.setTargetPosition(pos);
                        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        carousel.setPower(-0.7);
                    } else if (carousel.getCurrentPosition() == pos) {
                        carousel.setPower(0);
                    } else {
                        carousel.setTargetPosition(pos);
                        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        carousel.setPower(0.7);
                    }
                }
                count++;
            }

            if (slide.getCurrentPosition()-num >= 0 && carousel.getCurrentPosition()-pos >= 0) {
                slide.setPower(0.2);
                carousel.setPower(0);
                break;
            }
        }
    }

    void setAngle(double angle)
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

            if (carousel.getCurrentPosition()-pos >= 0) {
                carousel.setPower(0);
                break;
            }
        }
    }

    void setMotorPos(double angle, double height)
    {
        int pos = (int)(ppr*angle/360.0);
        while (opModeIsActive()) {
            if (Math.abs(getCarouselAngle()) < 30) {
                if (slide.getCurrentPosition() > height) {
                    slide.setTargetPosition((int) height);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                } else if (slide.getCurrentPosition() == 0) {
                    slide.setPower(0.2);
                } else {
                    slide.setTargetPosition((int) height);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                }
            }

            if (carousel.getCurrentPosition() > pos) {
                carousel.setTargetPosition(pos);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(-0.7);
            }
            else if (carousel.getCurrentPosition() == pos)
            {
                carousel.setPower(0);
            }
            else {
                carousel.setTargetPosition(pos);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(0.7);
            }

            if (slide.getCurrentPosition()-height <= 10 && carousel.getCurrentPosition()-pos <= 5) {
                slide.setPower(0.2);
                carousel.setPower(0);
                break;
            }
        }
    }

    void setMotorPosExtend(double angle, double height, double servoPos)
    {
        int pos = (int)(ppr*angle/360.0);
        while (opModeIsActive()) {
            if (Math.abs(getCarouselAngle()) < 30) {
                if (slide.getCurrentPosition() > height) {
                    slide.setTargetPosition((int) height);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                } else if (slide.getCurrentPosition() == 0) {
                    slide.setPower(0.2);
                } else {
                    slide.setTargetPosition((int) height);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                }
            }

            if (carousel.getCurrentPosition() > pos) {
                carousel.setTargetPosition(pos);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(-0.7);
            }
            else if (carousel.getCurrentPosition() == pos)
            {
                carousel.setPower(0);
            }
            else {
                carousel.setTargetPosition(pos);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(0.7);
            }

            if (Math.abs(getCarouselAngle()) <= 30)
                slideServo.setPosition(servoPos);

            if (slide.getCurrentPosition()-height <= 10 && carousel.getCurrentPosition()-pos <= 5) {
                slide.setPower(0.2);
                carousel.setPower(0);
                break;
            }
        }
    }

    double getCarouselAngle()
    {
        return (carousel.getCurrentPosition()*360.0/ppr);
    }

    void setMotorPos3(double angle, double height)
    {
        int pos = (int)(ppr*angle/360.0);
        while (opModeIsActive()) {
            if (slide.getCurrentPosition() > height) {
                slide.setTargetPosition((int) height);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-0.8);
            }
            else {
                slide.setTargetPosition((int) height);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            }

            if (slide.getCurrentPosition() > 500) {
                if (carousel.getCurrentPosition() > (int) (ppr * angle / 360.0)) {
                    carousel.setTargetPosition(pos);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(-0.7);
                } else if (carousel.getCurrentPosition() < (int) (ppr * angle / 360.0)) {
                    carousel.setTargetPosition(pos);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(0.7);
                }
                else
                    carousel.setPower(0);
            }

            if (Math.abs(slide.getCurrentPosition()-height) >= -5 && carousel.getCurrentPosition()-pos >= -5 && carousel.getCurrentPosition()-pos <= 5) {
                slide.setPower(0.2);
                carousel.setPower(0);
                break;
            }
        }
    }

    void setMotorPos4(double angle, double height)
    {
        int pos = (int)(ppr*angle/360.0);
        while (opModeIsActive()) {
            if (slide.getCurrentPosition() > height) {
                slide.setTargetPosition((int) height);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-0.8);
            }
            else if (slide.getCurrentPosition() < height) {
                slide.setTargetPosition((int) height);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            }
            else
                slide.setPower(0.2);

            if (slide.getCurrentPosition() > 500) {
                if (carousel.getCurrentPosition() > (int) (ppr * angle / 360.0)) {
                    carousel.setTargetPosition(pos);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(-0.7);
                } else if (carousel.getCurrentPosition() < (int) (ppr * angle / 360.0)) {
                    carousel.setTargetPosition(pos);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(0.7);
                }
                else
                    carousel.setPower(0);
            }

            if (Math.abs(slide.getCurrentPosition()-height) <= 10 && carousel.getCurrentPosition()-pos >= -5) {
                slide.setPower(0.2);
                carousel.setPower(0);
                break;
            }
        }
    }

    public void setMotorPos4Async(double angle, double height)
    {
        int pos = (int)(ppr*angle/360.0);
        if (!exitTrajAsync) {
            if (slide.getCurrentPosition() > height) {
                slide.setTargetPosition((int) height);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-0.8);
            } else if (slide.getCurrentPosition() < height) {
                slide.setTargetPosition((int) height);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            } else
                slide.setPower(0.2);

            if (slide.getCurrentPosition() > 500) {
                if (carousel.getCurrentPosition() > (int) (ppr * angle / 360.0)) {
                    carousel.setTargetPosition(pos);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(-0.7);
                } else if (carousel.getCurrentPosition() < (int) (ppr * angle / 360.0)) {
                    carousel.setTargetPosition(pos);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(0.7);
                } else
                    carousel.setPower(0);
            }
        }

        if (Math.abs(slide.getCurrentPosition()-height) <= 10 && carousel.getCurrentPosition()-pos >= -5) {
            slide.setPower(0.2);
            carousel.setPower(0);
            exitTrajAsync = true;
        }
    }

    void raiseHeight2(int num)
    {
        slide.setTargetPosition(num+zeroPos);
        if (slide.getCurrentPosition() > num+zeroPos)
            slide.setPower(-0.4);
        else
            slide.setPower(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slide.isBusy()) {

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
