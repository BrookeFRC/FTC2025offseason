package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Config
@TeleOp
public class PIDF_Arm extends OpMode {
    private PIDController controller;
    public  static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 50;
    private DcMotor port0;
    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        port0 = hardwareMap.get(DcMotor.class, "port0");

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = port0.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ticks_in_degree = (double) 252 /360;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
        port0.setPower(power);
        telemetry.addData("power", power);
        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
