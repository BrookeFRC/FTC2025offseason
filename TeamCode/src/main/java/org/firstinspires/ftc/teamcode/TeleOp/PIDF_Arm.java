package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Config
@TeleOp
public class PIDF_Arm extends OpMode {
    private PController sController;
    private PDController aController;
    public static double  sF = 0;
    public static int starget = -4100, atarget = 0, aF = 0;
    private DcMotor slide;
    private DcMotor arm;


    @Override
    public void init() {
        sController = new PController(0.019);
        aController = new PDController(0.03, 0.00009);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slide = hardwareMap.get(DcMotor.class, "slide");
        arm   = hardwareMap.get(DcMotor.class, "arm");

    }

    @Override
    public void loop() {
        //slide bounds are -4100 in and 4500 out
        sController.setP(0.019);
        slide.setPower(sController.calculate(slide.getCurrentPosition(), starget) + Math.cos(Math.toRadians(starget / (28*4) /360/*ticks in degree in the slide*/)) * sF); //do not touch the dark magic math (https://www.youtube.com/watch?v=E6H6Nqe6qJo
        //-550 is up all the way and 840 all the way down
        aController.setP(0.03);
        aController.setD(0.00009);
        arm.setPower(aController.calculate(arm.getCurrentPosition(), atarget) + Math.cos(Math.toRadians(atarget / (2800/360/*ticks in degree for arm*/))) * aF);// do not touch the math dark magic (https://www.youtube.com/watch?v=E6H6Nqe6qJo)

        telemetry.addData("arm pos", arm.getCurrentPosition());
        telemetry.addData("arm target ", atarget);
        telemetry.addData("slide pos", slide.getCurrentPosition());
        telemetry.addData("slide target ", starget);
        telemetry.update();
        //  TODO add feedforward based on slide extension


        //double sPid = sController.calculate(slide.getCurrentPosition(), starget);
        //double ticks_in_degree_slide = (double) ;
        //double sFF = Math.cos(Math.toRadians(starget / (28*4) /360/*ticks in degree in the slide*/)) * sF;
        //double spower = sPid + Math.cos(Math.toRadians(starget / (28*4) /360/*ticks in degree in the slide*/)) * sF;
        //telemetry.addData("slide power", slidepower);
        // double armPid = aController.calculate(arm.getCurrentPosition(), atarget);
        //double aPower = aController.calculate(arm.getCurrentPosition(), atarget) + Math.cos(Math.toRadians(atarget / (2800/360/*ticks in degree*/))) * aF;
        //telemetry.addData("arm power", aPower);



    }
}
