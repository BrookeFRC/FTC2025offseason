package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class FieldCentricDrive extends OpMode {
    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor slide;
    DcMotor arm;

    PController sController;
    PDController aController;
    public static double  sF = 0;
    public static int sTarget = -4100, aTarget = 0, aF = 0;


    @Override
    public void init() {
        //pid
        sController = new PController(0.019);
        aController = new PDController(0.03, 0.00009);
        // Declaring our motors and Retrieving the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
        frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        backRightMotor = hardwareMap.dcMotor.get("right_back_drive");
        slide = hardwareMap.get(DcMotor.class, "slide");
        arm   = hardwareMap.get(DcMotor.class, "arm");
        //telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;



        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {imu.resetYaw();}
        else if (gamepad1.a) {sTarget = 5400;}
        else if (gamepad1.b) {sTarget = -4100;}
        else if (gamepad1.x) {aTarget = 840;}
        else if (gamepad1.y) {aTarget = -550;}


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        //slide bounds are -4100 in and 4500 out
        sController.setP(0.019);
        slide.setPower(sController.calculate(slide.getCurrentPosition(), sTarget) + Math.cos(Math.toRadians((double) sTarget / (28*4) /360/*ticks in degree in the slide*/)) * sF); //do not touch the dark magic math (https://www.youtube.com/watch?v=E6H6Nqe6qJo
        //-550 is up all the way and 840 all the way down
        aController.setP(0.03);
        aController.setD(0.00009);
        arm.setPower(aController.calculate(arm.getCurrentPosition(), aTarget) + Math.cos(Math.toRadians((double) aTarget / ((double) 2800 /360/*ticks in degree for arm*/))) * aF);// do not touch the math dark magic (https://www.youtube.com/watch?v=E6H6Nqe6qJo)

        telemetry.addData("arm pos", arm.getCurrentPosition());
        telemetry.addData("arm target ", aTarget);
        telemetry.addData("slide pos", slide.getCurrentPosition());
        telemetry.addData("slide target ", sTarget);
        telemetry.update();
        // TODO add feedforward based on slide extension
        // TODO add inputs for slide in/out
        // TODO add inputs for claw (probably enums) (open and closed)
        // TODO add inputs for arm (down, up, middle)
        // TODO add webcam






    }

}

