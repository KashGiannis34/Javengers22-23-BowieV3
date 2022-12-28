package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp (name = "PIDF_Arm")
public class PIDF_Arm extends OpMode {
    private PIDFController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 2786.2/180.0;

    private DcMotorEx armMotor;

    Telemetry telemetry;

    @Override public void init()
    {
        controller = new PIDFController(p,i,d,f);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
    }

    @Override public void loop()
    {
        controller.setPIDF(p,i,d,f);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree))*f;
        double power = pid*ff;
        armMotor.setPower(power);

        telemetry.addData("pos: ", armPos);
        telemetry.addData("target: ", target);
        telemetry.update();
    }
}
