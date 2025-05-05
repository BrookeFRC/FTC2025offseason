package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@TeleOp(name = "TeleopMiracleTest")
public class TeleopMiracleTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {


                telemetry.update();
                }
            }
        }
    }


