package net.maxdev.ftc.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.maxdev.ftc.utils.MecanumWheels;

@TeleOp(name = "EncoderTest", group = "Debugging")
public class EncoderTest extends LinearOpMode {
    private MecanumWheels wheels = null;
    private ElapsedTime runtime = null;

    @Override
    public void runOpMode() {
        wheels = new MecanumWheels();
        wheels.init(telemetry, hardwareMap);

        runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (gamepad1.a && runtime.seconds() > 0.75) {
                runtime.reset();
                wheels.encoderDrive(8, 8, 8, 8, 1);
            }
        }
    }
}
