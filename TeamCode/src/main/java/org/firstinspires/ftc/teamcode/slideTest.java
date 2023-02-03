package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@TeleOp(name="Slide Test", group="Linear Opmode")
//@Disabled

public class slideTest extends LinearOpMode {
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

    final double OPEN = 0.19;
    final double CLOSE = 1.00;

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

        arm = hardwareMap.dcMotor.get("armMotor");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            slide();
            telemetry.addData("slide power: ", arm.getPower());
            telemetry.addData("slide pos: ", arm.getCurrentPosition());
            telemetry.update();

        }

    }

    public void slide()
    {
        arm.setPower(-1*gamepad1.left_stick_y);
    }



}


