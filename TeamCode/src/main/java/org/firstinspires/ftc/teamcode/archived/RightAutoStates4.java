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
import org.firstinspires.ftc.teamcode.Brobot;
import org.firstinspires.ftc.teamcode.barcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Right Auto States 4")
@Disabled
public class RightAutoStates4 extends LinearOpMode {
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
        TRAJ_1
    }
    State currState = State.TRAJ_1;

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

        Pose2d startPose = new Pose2d(32,-62,Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(34,-54), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(42,-11), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(32+(18/(52/9.0)),-44, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(41,-10, Math.toRadians(0)))
                .lineTo(new Vector2d(58, -10))
                .build();

        TrajectorySequence parkRight = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(61, -10))
                .build();

        TrajectorySequence parkMiddle = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(38, -10))
                .build();

        TrajectorySequence parkLeft = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineToConstantHeading(new Vector2d(13, -10))
                .build();

        TrajectorySequence moveToJunction = robot.trajectorySequenceBuilder(trajSeq.end())
                .lineTo(new Vector2d(25, -10))
                .UNSTABLE_addTemporalMarkerOffset(0, () ->
                {
                    raiseHeightAndServoAndAngle(1950, 0.2, 130, 470);
                })
                .waitSeconds(2)
                .build();

        claw.setPosition(1);
        slideServo.setPosition(0);
        sleep(100);
        raiseHeight2(50);
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

        setMotorPos3(-28, 1170);
        slideServo.setPosition(0.47);
        sleep(500);
        claw.setPosition(0.4);
        sleep(100);
        slideServo.setPosition(0);
        setMotorPos3(0, 375);

        robot.followTrajectorySequence(trajSeq);
        claw.setPosition(0.4);

        int[] stackHeights = {420, 342, 245, 158, 70};
        for (int n = 0; n < 1; n++) {
            if (etime.seconds() >= 24)
                break;
            setMotorPos(0, stackHeights[n]);
            slideServo.setPosition(0.2);
            sleep(300);
            claw.setPosition(1);
            sleep(350);
            robot.followTrajectorySequence(moveToJunction);
        }

        double start = System.currentTimeMillis();
        claw.setPosition(0.4);
        slideServo.setPosition(0);
        if (tagOfInterest == null || tagOfInterest.id == RIGHT)
            robot.followTrajectorySequence(parkRight);
        else if (tagOfInterest.id == MIDDLE)
            robot.followTrajectorySequence(parkMiddle);
        else if (tagOfInterest.id == LEFT)
            robot.followTrajectorySequence(parkLeft);
        else
            robot.followTrajectorySequence(parkRight);

        if (System.currentTimeMillis()-start >= 400)
            setMotorPos(0,10);

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
                slide.setPower(-0.8);
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
                        carousel.setPower(-0.4);
                    } else if (carousel.getCurrentPosition() == pos) {
                        carousel.setPower(0);
                    } else {
                        carousel.setTargetPosition(pos);
                        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        carousel.setPower(0.4);
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
                    slide.setPower(-0.8);
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
                carousel.setPower(-0.4);
            }
            else if (carousel.getCurrentPosition() == pos)
            {
                carousel.setPower(0);
            }
            else {
                carousel.setTargetPosition(pos);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(0.4);
            }

            if (slide.getCurrentPosition()-height <= 10 && carousel.getCurrentPosition()-pos <= 0) {
                slide.setPower(0.2);
                carousel.setPower(0);
                break;
            }
        }
    }

    public void setMotorPosIf(double angle, double height)
    {
        if (!exit) {
            int pos = (int) (ppr * angle / 360.0);
            if (Math.abs(getCarouselAngle()) < 30) {
                if (slide.getCurrentPosition() > height) {
                    slide.setTargetPosition((int) height);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-0.8);
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
                carousel.setPower(-0.4);
            } else if (carousel.getCurrentPosition() == pos) {
                carousel.setPower(0);
            } else {
                carousel.setTargetPosition(pos);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(0.4);
            }

            if (slide.getCurrentPosition() - height <= 10 && carousel.getCurrentPosition() - pos <= 0) {
                slide.setPower(0.2);
                carousel.setPower(0);
                exit = true;
            }
        }
    }

    double getCarouselAngle()
    {
        return (carousel.getCurrentPosition()*360.0/ppr);
    }

    void setMotorPos2(double angle, double height)
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

            if (slide.getCurrentPosition()-height >= 0 && carousel.getCurrentPosition()-pos >= -2 && carousel.getCurrentPosition()-pos <= 2) {
                slide.setPower(0.2);
                carousel.setPower(0);
                break;
            }
        }
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
                } else {
                    carousel.setTargetPosition(pos);
                    carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carousel.setPower(0.7);
                }
            }

            if (slide.getCurrentPosition()-height >= 0 && carousel.getCurrentPosition()-pos >= -2 && carousel.getCurrentPosition()-pos <= 2) {
                slide.setPower(0.2);
                carousel.setPower(0);
                break;
            }
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
