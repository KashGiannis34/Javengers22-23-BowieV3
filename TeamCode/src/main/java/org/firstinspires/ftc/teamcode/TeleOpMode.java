package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


@TeleOp(name="New TeleOp 2", group="Linear Opmode")
//@Disabled

public class TeleOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Hardware hardware;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;

    final double rpm = 1150/28.0;
    final double ppr = 145.1*28;
    boolean release = false;

    // operational constants
    double joyScale = 0.6;
    double motorMax = 0.6; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    public DcMotor lf, rf, lr, rr, arm, carousel;
    public Servo claw, slideServo;

    public enum Height
    {
        NONE,
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        STACK
    }

    Height level = Height.NONE;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.dcMotor.get("leftFront");
        lr = hardwareMap.dcMotor.get("leftBack");
        rf = hardwareMap.dcMotor.get("rightFront");
        rr = hardwareMap.dcMotor.get("rightBack");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        carousel = hardwareMap.dcMotor.get("turnTable");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.dcMotor.get("armMotor");
        claw = hardwareMap.servo.get("clawServo");
        slideServo = hardwareMap.servo.get("slideServo");
        slideServo.setPosition(0.1);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        double start = runtime.milliseconds();
        while (runtime.milliseconds() - start < 300)
            arm.setPower(-0.2);

        arm.setPower(0);
        sleep(10);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive();
            ground();
            low();
            medium();
            high();
            release();
            clawMove();
            stack();
            setSlideServo();
            setTurnTable();
            telemetry.addData("arm pos: ", arm.getCurrentPosition());
            telemetry.addData("level: ", level);
            telemetry.addData("slide servo: ", slideServo.getPosition());
            telemetry.addData("leftFront power: ", lf.getPower());
            if (arm.getCurrentPosition() <= 1000)
            {
                telemetry.addLine("arm not high enough");
            }
            else
            {
                telemetry.addLine("arm high enough");
            }
            telemetry.update();

        }

    }

    public void drive()
    {
        // Reset speed variables
        LF = 0;
        RF = 0;
        LR = 0;
        RR = 0;

        // Get joystick values
        Y1 = -gamepad1.left_stick_y * joyScale; // invert so up is positive
        X1 = gamepad1.left_stick_x * joyScale;
        Y2 = -gamepad1.right_stick_y * joyScale; // Y2 is not used at present
        X2 = gamepad1.right_stick_x * joyScale;

        // Forward/back movement
        LF += Y1;
        RF += Y1;
        LR += Y1;
        RR += Y1;

        // Side to side movement
        LF += X1;
        RF -= X1;
        LR -= X1;
        RR += X1;

        // Rotation movement
        LF += X2;
        RF -= X2;
        LR += X2;
        RR -= X2;



        // Clip motor power values to +-motorMax
        LF = Math.max(-motorMax, Math.min(LF, motorMax));
        RF = Math.max(-motorMax, Math.min(RF, motorMax));
        LR = Math.max(-motorMax, Math.min(LR, motorMax));
        RR = Math.max(-motorMax, Math.min(RR, motorMax));

        // Send values to the motors
        lf.setPower(LF);
        rf.setPower(RF);
        lr.setPower(LR);
        rr.setPower(RR);
    }

    public void ground()
    {
        if (gamepad2.dpad_down) {
            level = Height.GROUND;
            if (arm.getCurrentPosition() > 100) {
                arm.setTargetPosition(100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
            } else {
                arm.setTargetPosition(100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

    public void low()
    {
        if (gamepad2.dpad_left) {
            level = Height.LOW;
            if (arm.getCurrentPosition() > 1250) {
                arm.setTargetPosition(1250);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
            } else {
                arm.setTargetPosition(1250);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

    public void medium()
    {
        if (gamepad2.dpad_up) {
            level = Height.MEDIUM;
            if (arm.getCurrentPosition() > 2100) {
                arm.setTargetPosition(2100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
            } else {
                arm.setTargetPosition(2100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

    public void high()
    {
        if (gamepad2.dpad_right) {
            level = Height.HIGH;
            if (arm.getCurrentPosition() > 2900) {
                arm.setTargetPosition(2900);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
            } else {
                arm.setTargetPosition(2900);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

    public void clawMove()
    {
        if (gamepad1.a) {
            claw.setPosition(0.19);
            if (level == Height.GROUND) {
                arm.setPower(0);
                level = Height.NONE;
            }
        }
        if (gamepad1.y) {
            claw.setPosition(1.0);
            if (level == Height.NONE) {
                level = Height.GROUND;
                if (arm.getCurrentPosition() > 30) {
                    arm.setTargetPosition(30);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(-1);
                } else {
                    arm.setTargetPosition(30);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                }
            }
        }
    }

    public void release()
    {
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            arm.setPower(0);
        }
    }

    public void stack()
    {
        if (gamepad2.y) {
            level = Height.STACK;
            if (arm.getCurrentPosition() > 380) {
                arm.setTargetPosition(380);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-0.7);
            } else {
                arm.setTargetPosition(380);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
            }
        }

        if (gamepad2.b) {
            level = Height.STACK;
            if (arm.getCurrentPosition() > 288) {
                arm.setTargetPosition(288);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-0.7);
            } else {
                arm.setTargetPosition(288);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
            }
        }

        if (gamepad2.a) {
            level = Height.STACK;
            if (arm.getCurrentPosition() > 195) {
                arm.setTargetPosition(195);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-0.7);
            } else {
                arm.setTargetPosition(195);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
            }
        }

        if (gamepad2.x) {
            level = Height.STACK;
            if (arm.getCurrentPosition() > 103) {
                arm.setTargetPosition(103);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-0.7);
            } else {
                arm.setTargetPosition(103);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
            }
        }
    }

    public void setAngle(int angle)
    {
        int pos = (int)(ppr*angle/360.0);
        if (carousel.getCurrentPosition() > pos) {
            carousel.setTargetPosition(pos);
            carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            carousel.setPower(-0.7);
        } else {
            carousel.setTargetPosition(pos);
            carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            carousel.setPower(0.7);
        }
    }

    public void setTurnTable()
    {
        if (gamepad1.dpad_left && arm.getCurrentPosition() > 1000)
        {
            setAngle(-90);
        }
        if (gamepad1.dpad_right && arm.getCurrentPosition() > 1000)
        {
            setAngle(90);
        }
        if (gamepad1.dpad_up && arm.getCurrentPosition() > 1000)
        {
            setAngle(0);
        }
    }

    public void setSlideServo()
    {
        if (gamepad1.x)
        {
            slideServo.setPosition(0);
        }
        if (gamepad1.b)
        {
            slideServo.setPosition(1);
        }
    }

}


