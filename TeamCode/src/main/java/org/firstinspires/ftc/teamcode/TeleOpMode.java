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
    boolean gamepad1AisPressed = false;
    boolean gamepad2AisPressed = false;
    boolean gamepad1YisPressed = false;
    int stackCount = -1;
    boolean angleChanged  = false;
    double startTime;

    private List<DcMotor> motors;

    final double ppr = 1992.6;

    // operational constants
    double joyScale = 0.6;
    double motorMax = 0.6; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    public DcMotor lf, rf, lr, rr, arm, carousel;
    public Servo claw, slideServo;
    public RevBlinkinLedDriver lights;

    final double OPEN = 0.4;
    final double CLOSE = 1;
    public static boolean clawClosed = false;
    public static int slideExtended = 0;

    public enum Height
    {
        NONE,
        GROUND,
        CLAWUP,
        LOW,
        MEDIUM,
        HIGH,
        STACK5,
        STACK4,
        STACK3,
        STACK2
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
        setAngle(0);
        sleep(10);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive();
            release();
            low();
            medium();
            high();
            clawMove();
            stack();
            setTurnTable();
            setSlideServo();
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

            telemetry.addData("gamepad2 x: ", gamepad2.x);
            telemetry.addData("arm pos: ", arm.getCurrentPosition());
            telemetry.addData("level: ", level);
            telemetry.addData("stackCount: ", stackCount);
            telemetry.addData("slide servo: ", slideServo.getPosition());
            telemetry.addData("leftFront power: ", lf.getPower());
            telemetry.addData("time elapsed (seconds): ", runtime.seconds());
            telemetry.addData("angleChanged: ", angleChanged);
            telemetry.addData("heading: ", getAngle());
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

    public void release()
    {
        if (gamepad2.dpad_down && !angleChanged) {
            level = Height.NONE;
            raiseHeight(5);
        }

        if (gamepad2.dpad_down && angleChanged)
        {
            level = Height.NONE;
            setAngle(0);
            angleChanged = false;
            if (slideServo.getPosition() != 1) {
                slideExtended = 0;
            }
        }

        if (Math.abs(carousel.getCurrentPosition()) <= 100 && level == Height.NONE)
            raiseHeight(5);
    }

    public void low()
    {
        if (gamepad2.dpad_left) {
            level = Height.LOW;
            raiseHeight(1180);
        }
    }

    public void medium()
    {
        if (gamepad2.dpad_up) {
            level = Height.MEDIUM;
            raiseHeight(2070);
        }
    }

    public void high()
    {
        if (gamepad2.dpad_right) {
            level = Height.HIGH;
            raiseHeight(2820);
        }
    }

    public void clawMove()
    {
        if (gamepad1.a && !gamepad1AisPressed)
        {
            if (clawClosed)
                clawClosed = false;
            else
                clawClosed = true;

            gamepad1AisPressed = true;
            startTime = runtime.milliseconds();
        }

        if (gamepad1AisPressed && !gamepad1.a)
            gamepad1AisPressed = false;

        if (!clawClosed) {
            claw.setPosition(OPEN);
            if (overrides == LightOverride.NONE)
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            if (level == Height.CLAWUP) {
                raiseHeight(5);
                level = Height.NONE;
            }
        }
        else {
            claw.setPosition(CLOSE);
            if (overrides == LightOverride.NONE)
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            if (level == Height.NONE) {
                level = Height.CLAWUP;
            }

            telemetry.addData("claw time: ",runtime.milliseconds() - startTime);
            if (runtime.milliseconds()-startTime >= 300 && level == Height.CLAWUP)
                raiseHeight(50);
        }
    }

    public void stack()
    {
        if (gamepad2.a && !gamepad2AisPressed && !angleChanged)
        {
            if (stackCount == -1)
                stackCount = 0;
            else
                stackCount++;

            if (stackCount % 4 == 0)
                level = Height.STACK5;
            else if (stackCount % 4 == 1)
                level = Height.STACK4;
            else if (stackCount % 4 == 2)
                level = Height.STACK3;
            else
                level = Height.STACK2;


            gamepad2AisPressed = true;
            startTime = runtime.milliseconds();
        }

        if (gamepad2AisPressed && !gamepad2.a)
            gamepad2AisPressed = false;

        if (!(level == Height.STACK2 || level == Height.STACK3 || level == Height.STACK4 || level == Height.STACK5))
            stackCount = -1;

        if (stackCount % 4 == 0 && !angleChanged)
            raiseHeight(380);
        else if (stackCount % 4 == 1 && !angleChanged)
            raiseHeight(288);
        else if (stackCount % 4 == 2 && !angleChanged)
            raiseHeight(195);
        else if (stackCount % 4 == 3 && stackCount != -1 && !angleChanged)
            raiseHeight(103);
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
        if (gamepad2.x && (arm.getCurrentPosition() > 1000 || (level == Height.LOW || level == Height.MEDIUM || level == Height.HIGH)))
        {
            angleChanged = true;
            setAngle(-90);
            if (slideServo.getPosition() != 1) {
                slideExtended = -1;
            }
        }
        if (gamepad2.b && (arm.getCurrentPosition() > 1000 || (level == Height.LOW || level == Height.MEDIUM || level == Height.HIGH)))
        {
            angleChanged = true;
            setAngle(90);
            if (slideServo.getPosition() != 1) {
                slideExtended = -1;
            }
        }
        if (gamepad2.y && (arm.getCurrentPosition() > 1000 || (level == Height.LOW || level == Height.MEDIUM || level == Height.HIGH)))
        {
            angleChanged = false;
            setAngle(0);
            if (slideServo.getPosition() != 1) {
                slideExtended = 0;
            }
        }
    }

    public void setSlideServo()
    {
        if (gamepad1.y && !gamepad1YisPressed)
        {
            if (slideExtended != 0)
                slideExtended = 0;
            else if (slideExtended == 0)
                slideExtended = 1;

            gamepad1YisPressed = true;
        }

        if (gamepad1YisPressed && !gamepad1.y)
            gamepad1YisPressed = false;

        if (slideExtended == 0)
        {
            slideServo.setPosition(0.1);
        }
        else if (slideExtended == 1)
        {
            slideServo.setPosition(1);
        }
        else if (slideExtended == -1 && Math.abs(arm.getTargetPosition()-arm.getCurrentPosition()) <= 10)
        {
            slideServo.setPosition(0.34);
        }
        else if (slideExtended == -2)
            slideServo.setPosition(0.31);
        else if (slideExtended == -3)
            slideServo.setPosition(0.37);


        if (gamepad2.left_bumper && (slideServo.getPosition() == 0.34 || slideServo.getPosition() == 0.37))
            slideExtended = -2;
        if (gamepad2.right_bumper && (slideServo.getPosition() == 0.31 || slideServo.getPosition() == 0.34))
            slideExtended = -3;
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

    public void turn(double degrees)
    {
        resetAngle();

        double error = degrees;

        double motorPower = 0;
        while (Math.abs(error) > 2)
        {
            motorPower = (error < 0 ? -1*gamepad1.right_stick_x:gamepad1.right_stick_x);
            error = degrees - getAngle();
            telemetry.addData("error: ", error);
        }
        motorPower = 0;
    }

}


