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
                wheels.encoderDrive(8, 8, 8, 8, 0.7, 2);
            }
            if (gamepad1.dpad_up && runtime.seconds() > 0.75) {
                runtime.reset();
                wheels.P_DRIVE_COEFF = wheels.P_DRIVE_COEFF + 0.01;
            }
            if (gamepad1.dpad_down && runtime.seconds() > 0.75) {
                runtime.reset();
                wheels.P_DRIVE_COEFF = wheels.P_DRIVE_COEFF - 0.01;
            }
            wheels.debug();
            telemetry.update();
        }
    }
}
