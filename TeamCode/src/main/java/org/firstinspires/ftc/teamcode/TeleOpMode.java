package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    public int zeropos;

    // operational constants
    double joyScale = 0.6;
    double motorMax = 0.6; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    public DcMotor lf, rf, lr, rr, arm;
    public Servo armServo, turnTable, claw;
    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.dcMotor.get("leftFront");
        lr = hardwareMap.dcMotor.get("leftBack");
        rf = hardwareMap.dcMotor.get("rightFront");
        rr = hardwareMap.dcMotor.get("rightBack");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.dcMotor.get("armMotor");
        armServo = hardwareMap.servo.get("armServo");
        claw = hardwareMap.servo.get("clawServo");

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
            arm.setPower(-0.5);

        arm.setPower(0);
        zeropos = arm.getCurrentPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.update();

            drive();
            low();
            medium();
            high();
            release();
            clawMove();
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

    public void low()
    {
        if (gamepad2.dpad_down) {
            armServo.setPosition(0.1);
            if (arm.getCurrentPosition() > 1000) {
                arm.setTargetPosition(1000);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
            } else {
                arm.setTargetPosition(1000);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

    public void medium()
    {
        if (gamepad2.dpad_left) {
            armServo.setPosition(0.1);
            if (arm.getCurrentPosition() > 1700) {
                arm.setTargetPosition(1700);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
            } else {
                arm.setTargetPosition(1700);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

    public void high()
    {
        if (gamepad2.dpad_up) {
            armServo.setPosition(1);
            if (arm.getCurrentPosition() > 1700) {
                arm.setTargetPosition(1700);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
            } else {
                arm.setTargetPosition(1700);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

    public void clawMove()
    {
        if (gamepad1.a) {
            claw.setPosition(0.6);
        }
        if (gamepad1.y) {
            claw.setPosition(1.0);
        }
    }

    public void release()
    {
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            armServo.setPosition(0.1);
            if (arm.getCurrentPosition() > zeropos+10) {
                arm.setTargetPosition(zeropos+10);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
            } else {
                arm.setTargetPosition(zeropos+10);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
            }
        }
    }

}


