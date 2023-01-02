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
    public boolean update = false;

    DcMotor armMotor;
    Servo clawServo, armServo, ttServo;

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
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawServo = hardwareMap.servo.get("clawServo");
        armServo = hardwareMap.servo.get("armServo");
        ttServo = hardwareMap.servo.get("turnTableServo");
    }

    @Override public void loop()
    {
        driveMove();
        turbo();
        openClaw();
        clawPatrol();
        updateJunction();
        reachTerminal();
        turnTable();
        moveArmServo();

    }

    public void openClaw()
    {
        if (gamepad1.x && closed)
            closed = false;
        else if (gamepad1.y && !closed)
            closed = true;
    }

    public void clawPatrol()
    {
        if (closed)
            clawServo.setPosition(0);
        else
            clawServo.setPosition(0.5);
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

    public void reachTerminal()
    {
        if (gamepad2.dpad_left)
        {
            armMotor.setPower(-1);
        }
        else
        {
            armMotor.setPower(0);
        }

        if (gamepad2.dpad_right)
        {
            armMotor.setPower(1);
        }
        else
        {
            armMotor.setPower(0);
        }
    }

    public void updateJunction()
    {
        if (gamepad2.y)
        {
            junctionLevel = Junction.GROUND;
            update = true;
        }
        else if (gamepad2.x)
        {
            junctionLevel = Junction.LOW;
            update = true;
        }
        else if (gamepad2.a)
        {
            junctionLevel = Junction.MEDIUM;
            update = true;
        }
        else if (gamepad2.b)
        {
            junctionLevel = Junction.HIGH;
            update = true;
        }
    }

    public void turnTable()
    {
        ttServo.setPosition(ttServo.getPosition()+(gamepad2.left_stick_x/100.0));
    }
    public void moveArmServo()
    {
        armServo.setPosition(armServo.getPosition()+(gamepad2.right_stick_x/100.0));
    }
}
