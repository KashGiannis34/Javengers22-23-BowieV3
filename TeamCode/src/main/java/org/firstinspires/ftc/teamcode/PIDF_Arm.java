package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@Autonomous (name = "PIDF_Arm")
public class PIDF_Arm extends LinearOpMode {
    private PIDFController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    public static int armServoPos = 0;

    private final double ticks_in_degree = 2786.2/180.0;

    private DcMotorEx armMotor;
    private Servo armServo;

    private FtcDashboard dashboard = FtcDashboard.getInstance();



    @Override public void runOpMode()
    {
        controller = new PIDFController(p,i,d,f);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armServo = hardwareMap.servo.get("armServo");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            controller.setPIDF(p, i, d, f);
            int armPos = armMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid * ff;
            armMotor.setPower(power);
            armServo.setPosition(armServoPos);

            telemetry.addData("pos: ", armPos);
            telemetry.addData("target: ", target);
            telemetry.update();
        }
    }
}
