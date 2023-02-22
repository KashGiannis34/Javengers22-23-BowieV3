package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "Sensor Test")
public class SensorTest extends LinearOpMode {
    Brobot robot;
    //ColorSensor color;
    DcMotor carousel;
    public DistanceSensor rightDist, leftDist;
    ElapsedTime etime = new ElapsedTime();
    final double rpm = 1150/28.0;
    final double ppr = 145.1*28;
    int cZeroPos, zeroPos;
    @Override
    public void runOpMode()
    {
        rightDist = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDistance");
        //color = hardwareMap.get(ColorSensor.class, "color");
        robot = new Brobot(hardwareMap);
        carousel = hardwareMap.dcMotor.get("turnTable");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            Pose2d poseEstimate = robot.getPoseEstimate();
            //telemetry.addData("red: ", color.red());
            //telemetry.addData("green: ", color.green());
            //telemetry.addData("blue: ", color.blue());
            telemetry.addData("Left Distance (cm): ", leftDist.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance (cm): ", rightDist.getDistance(DistanceUnit.CM));
            telemetry.addData("time elapsed: ", etime.seconds());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.addData("carousel pos: ", carousel.getCurrentPosition());
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) ;
    }



//    void raiseHeight(int num)
//    {
//        slide.setTargetPosition(num+zeroPos);
//        slide.setPower(1);
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (slide.isBusy()) {
//
//        }
//
//        slide.setPower(0.2);
//
//        //clawSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
}
