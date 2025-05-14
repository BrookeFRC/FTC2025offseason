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
    private PIDController slideController;
    public static double slideP = 0.019, slideI = 0, slideD = 0, slideF = 0;
    public static int slidetarget = -4100;
    private DcMotor slide;

    private PIDController armController;
    public static double armP = 0.03, armI = 0, armD = 0.00009, armF = 0;
    public static int armtarget = 0;
    private DcMotor arm;

    @Override
    public void init() {
        slideController = new PIDController(slideP,slideI,slideD);
        armController = new PIDController(armP,armI,armD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slide = hardwareMap.get(DcMotor.class, "slide");
        arm   = hardwareMap.get(DcMotor.class, "arm");

    }

    @Override
    public void loop() {
        //slide bounds are -4100 in and 4500 out


        slideController.setPID(slideP, slideI, slideD);
        int slideCurrentPosition = slide.getCurrentPosition();
        double slidePid = slideController.calculate(slideCurrentPosition, slidetarget);
        double ticks_in_degree = (double) (28*4) /360;
        double slideFF = Math.cos(Math.toRadians(slidetarget / ticks_in_degree)) * slideF;
        double slidepower = slidePid + slideFF;
        slide.setPower(slidepower);
        //telemetry.addData("slide power", slidepower);
        telemetry.addData("slide pos", slideCurrentPosition);
        telemetry.addData("slide target ", slidetarget);



        //-550 is up all the way and 840 all the way down
        armController.setPID(armP, armI, armD);
        int armPos =  arm.getCurrentPosition();
        double armPid = armController.calculate(armPos, armtarget);
        double ticks_in_degree_arm = 2800/360;
        double armFF = Math.cos(Math.toRadians(armtarget / ticks_in_degree_arm)) * armF;
        double armPower = armPid +armFF;
        arm.setPower(armPower);
        //telemetry.addData("arm power", armPower);
        telemetry.addData("arm pos", armPos);
        telemetry.addData("arm target ", armtarget);
        //  TODO add feedforward based on slide extension
        telemetry.update();




    }
}
