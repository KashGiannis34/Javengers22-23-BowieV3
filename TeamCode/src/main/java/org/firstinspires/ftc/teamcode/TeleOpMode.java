package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "TeleOp")
public class TeleOpMode extends OpMode {

    public boolean closed = true;
    public double maxSpeed = 0.65;
    public boolean lowSpeed = false;
    public boolean highSpeed = false;

    public enum Junction
    {
        NONE,
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }
    Junction junctionLevel = Junction.NONE;

    Brobot robot;

    @Override public void init()
    {
        robot = new Brobot(hardwareMap);
    }

    @Override public void loop()
    {
        driveMove();
        turbo();
    }

    public void driveMove()
    {
        robot.setWeightedDrivePower(new Pose2d(
                gamepad1.left_stick_y*maxSpeed*-1, gamepad1.left_stick_x*maxSpeed*-1, gamepad1.right_stick_x*maxSpeed*-1));
    }

    public void turbo()
    {
        if (maxSpeed > 0.1 && gamepad1.left_bumper)
            lowSpeed = true;

        if (gamepad1.right_bumper && maxSpeed < 1)
            highSpeed = true;

        if (lowSpeed && !gamepad1.left_bumper)
        {
            maxSpeed = 0.35;
            lowSpeed = false;
        }

        if (highSpeed && !gamepad1.right_bumper && junctionLevel != Junction.HIGH)
        {
            maxSpeed = 0.65;
            lowSpeed = false;
        }

        if (maxSpeed < 0.35)
            maxSpeed = 0.35;
        if (maxSpeed > 0.65)
            maxSpeed = 0.65;
    }
}
