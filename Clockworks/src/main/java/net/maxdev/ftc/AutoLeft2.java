package net.maxdev.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.maxdev.ftc.utils.DogeGoldVision;
import net.maxdev.ftc.utils.MecanumWheels;

@Autonomous(name = "AutoLeft", group = "Autonomous")
public class AutoLeft2 extends LinearOpMode {
    private MecanumWheels wheels = null;
    private DogeGoldVision detector = null;

    @Override
    public void runOpMode() {
        wheels = new MecanumWheels();
        wheels.init(telemetry, hardwareMap);
        detector.init(hardwareMap, telemetry);

        waitForStart();
        detector.enable(true);
        wheels.encoderDrive(8, 8, 8, 8, 0.6, 2);
    }
}
