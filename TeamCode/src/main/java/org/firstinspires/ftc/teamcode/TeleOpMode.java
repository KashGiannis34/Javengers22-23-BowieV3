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


@TeleOp(name="New TeleOp ", group="Linear Opmode")
//@Disabled

public class TeleOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Hardware hardware;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    // operational constants
    double joyScale = 0.6;
    double motorMax = 0.6; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    public DcMotor lf, rf, lr, rr, arm;
    public Servo armServo, turnTable, claw;
    @Override
    public void runOpMode() {
        lf = hardwareMap.dcMotor.get("leftFront");
        lr = hardwareMap.dcMotor.get("leftBack");
        rf = hardwareMap.dcMotor.get("rightFront");
        rr = hardwareMap.dcMotor.get("rightBack");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.dcMotor.get("armMotor");
        armServo = hardwareMap.servo.get("armServo");
        turnTable = hardwareMap.servo.get("turnTableServo");
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


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.update();


            // Reset speed variables
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;

            // Get joystick values
            Y1 = -gamepad1.right_stick_y * joyScale; // invert so up is positive
            X1 = gamepad1.right_stick_x * joyScale;
            Y2 = -gamepad1.left_stick_y * joyScale; // Y2 is not used at present
            X2 = gamepad1.left_stick_x * joyScale;

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

            ////////////////////////////////////////////////////////////////// NEW GAMEPAD 2
            if (gamepad1.a) {
                claw.setPosition(0.6);
            }
            if (gamepad1.y) {
                claw.setPosition(1.0);
            }

            if (gamepad1.x) {
                armServo.setPosition(0.0);
            }
            if (gamepad1.b) {
                armServo.setPosition(1.0);
            }



            if (gamepad2.dpad_down) {
                if (arm.getCurrentPosition() > 1300) {
                    arm.setTargetPosition(1300);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(-1);
                } else {
                    arm.setTargetPosition(1300);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                }
                arm.setPower(0.1);
            }

            if (gamepad2.dpad_right) {
                if (arm.getCurrentPosition() > 2000) {
                    arm.setTargetPosition(2000);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(-1);
                } else {
                    arm.setTargetPosition(2000);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                }
                arm.setPower(0.1);
            }

            if (gamepad2.a) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-0.2);
            }
            if (gamepad2.y) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(0.2);
            }
            if (gamepad2.x) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(0.0);
            }

//
//            if (gamepad2.a) {
//                hardware.collect.setPower(1.0);
//                /*
//                if (hardware.turnArm.getCurrentPosition() > 4500)
//                {
//                    hardware.turnArm.setTargetPosition(4500);
//                    hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    hardware.turnArm.setPower(-0.8);
//                }
//                else
//                {
//                    hardware.turnArm.setTargetPosition(4500);
//                    hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    hardware.turnArm.setPower(0.8);
//                }
//                */
//            }
//
//
//            if (gamepad2.x) {
//                hardware.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                hardware.arm.setPower(0.6);
//                //hardware.rotate.setPower(0.5);
//            }
//            if (gamepad2.b)
//            {
//                hardware.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                hardware.arm.setPower(-0.6);
//                //hardware.rotate.setPower(0);
//            }
//            if (gamepad2.y)
//            {
//                hardware.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                hardware.arm.setPower(0.0);
//            }
//            if (gamepad2.right_bumper)
//            {
//                motorMax=0.8;
//                joyScale=0.8;
//            }
//            if (gamepad2.left_bumper)
//            {
//                motorMax=1.0;
//                joyScale=1.0;
//            }
//            if (gamepad2.dpad_left)
//            {
//                hardware.rotate.setPower(0.5);
//            }
//            if (gamepad2.dpad_up)
//            {
//                hardware.rotate.setPower(0);
//            }
//            if (gamepad2.dpad_right)
//            {
//                hardware.rotate.setPower(-0.5);
//            }
//            if (gamepad1.dpad_left)
//            {
//                hardware.collect.setPower(1.0);
//            }
//            if (gamepad1.dpad_right)
//            {
//                hardware.collect.setPower(-1.0);
//            }
//            if (gamepad1.dpad_up)
//            {
//                hardware.collect.setPower(0.0);
//            }
//            if (gamepad1.x)
//            {
//                hardware.turnArm.setTargetPosition(6830);
//                hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.turnArm.setPower(0.8);
//            }
//            if (gamepad1.y)
//            {
//                if (hardware.turnArm.getCurrentPosition() > 4500)
//                {
//                    hardware.turnArm.setTargetPosition(4500);
//                    hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    hardware.turnArm.setPower(-0.8);
//                }
//                else
//                {
//                    hardware.turnArm.setTargetPosition(4500);
//                    hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    hardware.turnArm.setPower(0.8);
//                }
//
//            }
//            if (gamepad1.a)
//            {
//                if (hardware.turnArm.getCurrentPosition() > 5500)
//                {
//                    hardware.turnArm.setTargetPosition(5500);
//                    hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    hardware.turnArm.setPower(-0.8);
//                }
//                else
//                {
//                    hardware.turnArm.setTargetPosition(5500);
//                    hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    hardware.turnArm.setPower(0.8);
//                }
//
//            }
//            if (gamepad1.right_bumper)
//            {
//                if (hardware.turnArm.getCurrentPosition() > 6000)
//                {
//                    hardware.turnArm.setTargetPosition(6000);
//                    hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    hardware.turnArm.setPower(-0.8);
//                }
//                else
//                {
//                    hardware.turnArm.setTargetPosition(6000);
//                    hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    hardware.turnArm.setPower(0.8);
//                }
//
//            }
//
//            if (gamepad1.b)
//            {
//                hardware.turnArm.setTargetPosition(0);
//                hardware.turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.turnArm.setPower(-0.8);
//            }
        }

    }

}


