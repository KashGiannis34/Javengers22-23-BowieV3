package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;


@TeleOp(name="Robot Centric TeleOp", group="Linear Opmode")
//@Disabled

public class TeleOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Hardware hardware;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    private List<DcMotor> motors;

    final double rpm = 1150/28.0;
    final double ppr = 145.1*28;
    boolean release = false;

    // operational constants
    double joyScale = 0.6;
    double motorMax = 0.6; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    public DcMotor lf, rf, lr, rr, arm, carousel;
    public Servo claw, slideServo;
    public RevBlinkinLedDriver lights;

    final double OPEN = 0.19;
    final double CLOSE = 1.00;
    public static boolean clawClosed = false;
    public static boolean slideExtended = false;

    public enum Height
    {
        NONE,
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        STACK
    }

    public enum LightOverride
    {
        NONE,
        COUNTDOWN
    }

    LightOverride overrides = LightOverride.NONE;
    Height level = Height.NONE;
    public static boolean driveMode = true;

    @Override
    public void runOpMode() throws InterruptedException {

        lf = hardwareMap.dcMotor.get("leftFront");
        lr = hardwareMap.dcMotor.get("leftBack");
        rf = hardwareMap.dcMotor.get("rightFront");
        rr = hardwareMap.dcMotor.get("rightBack");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");


        motors = Arrays.asList(lf, lr, rr, rf);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // stops motors at 0 power
        for (DcMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMotorType(motorConfigurationType);
        }

        carousel = hardwareMap.dcMotor.get("turnTable");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm = hardwareMap.dcMotor.get("armMotor");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);

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
            setMotorType();
            if (runtime.seconds() >= 110)
            {
                overrides = LightOverride.COUNTDOWN;
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }

            if (runtime.seconds() >= 80 && runtime.seconds() <= 90)
            {
                overrides = LightOverride.COUNTDOWN;
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            }

            if (runtime.seconds() > 90 && runtime.seconds() < 110)
            {
                overrides = LightOverride.NONE;
                if (clawClosed)
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                else
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }

            telemetry.addData("arm pos: ", arm.getCurrentPosition());
            telemetry.addData("level: ", level);
            telemetry.addData("slide servo: ", slideServo.getPosition());
            telemetry.addData("leftFront power: ", lf.getPower());
            telemetry.addData("time elapsed (seconds): ", runtime.seconds());
            if (driveMode)
                telemetry.addLine("driveMode: brake");
            else
                telemetry.addLine("driveMode: no brake");
            if (arm.getCurrentPosition() <= 1000)
                telemetry.addLine("arm not high enough for turntable");
            else
                telemetry.addLine("arm high enough for turntable");
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
            raiseHeight(100);
        }
    }

    public void low()
    {
        if (gamepad2.dpad_left) {
            level = Height.LOW;
            raiseHeight(1250);
        }
    }

    public void medium()
    {
        if (gamepad2.dpad_up) {
            level = Height.MEDIUM;
            raiseHeight(2100);
        }
    }

    public void high()
    {
        if (gamepad2.dpad_right) {
            level = Height.HIGH;
            raiseHeight(2900);
        }
    }

    public void clawMove()
    {
        if (gamepad1.a) {
            claw.setPosition(OPEN);
            clawClosed = false;
            if (overrides == LightOverride.NONE)
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            if (level == Height.GROUND) {
                arm.setPower(0);
                level = Height.NONE;
            }
        }
        if (gamepad1.y) {
            claw.setPosition(CLOSE);
            clawClosed = true;
            if (overrides == LightOverride.NONE)
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            if (level == Height.NONE) {
                level = Height.GROUND;
                raiseHeight(30);
            }
        }
    }

    public void release()
    {
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            raiseHeight(10);
            level = Height.NONE;
        }
    }

    public void stack()
    {
        if (gamepad2.y) {
            level = Height.STACK;
            raiseHeight(380);
        }

        if (gamepad2.b) {
            level = Height.STACK;
            raiseHeight(288);
        }

        if (gamepad2.a) {
            level = Height.STACK;
            raiseHeight(195);
        }

        if (gamepad2.x) {
            level = Height.STACK;
            raiseHeight(103);
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
            slideServo.setPosition(0.1);
            slideExtended = false;
        }
        if (gamepad1.b)
        {
            slideServo.setPosition(1);
            slideExtended = true;
        }
    }

    public void raiseHeight(int height)
    {
        if (arm.getCurrentPosition() > height) {
            arm.setTargetPosition(height);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(-1);
        } else {
            arm.setTargetPosition(height);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
        }
    }

    public void setMotorType()
    {
        if (gamepad1.left_bumper)
        {
            driveMode = true;
            for (DcMotor motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMotorType(motorConfigurationType);
            }
        }

        if (gamepad1.right_bumper)
        {
            driveMode = false;
            for (DcMotor motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor.setMotorType(motorConfigurationType);
            }
        }

    }

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle()
    {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if (deltaAngle > 180)
            deltaAngle -= 360;
        else if (deltaAngle <= -180)
            deltaAngle += 360;

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro: ", orientation.firstAngle);

        return currAngle;
    }

}


