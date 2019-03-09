package net.maxdev.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.maxdev.ftc.utils.BackControl;
import net.maxdev.ftc.utils.DogeGoldVision;
import net.maxdev.ftc.utils.FrontControl;
import net.maxdev.ftc.utils.MecanumWheels;

public class AutoLeft2 extends LinearOpMode {
    private MecanumWheels wheels = null;
    private FrontControl front = null;
    private BackControl back = null;
    private DogeGoldVision detector = null;

    @Override
    public void runOpMode() {
        wheels.init(telemetry, hardwareMap);
        front.init(hardwareMap, telemetry);
        back.init(telemetry, hardwareMap);
        detector.init(hardwareMap, telemetry);

        waitForStart();

        detector.enable(true);
        wheels.encoderDrive(10, 10, 10, 10, 0.8, 3);

        if (detector.getLocation() > 200 && detector.getLocation() < 400) {
            detector.enable(false);
            wheels.encoderDrive(20, 20, 20, 20, 0.9, 3);
        } else {
            wheels.rotate(30, 0.8, 1, 4);
            if (detector.getLocation() > 200 && detector.getLocation() < 400) {
                detector.enable(false);
                wheels.encoderDrive(20, 20, 20, 20, 0.9, 3);
            } else {
                detector.enable(false);
                wheels.rotate(320, 0.9, 1, 5);
                wheels.encoderDrive(20, 20, 20, 20, 0.9,3);
            }
        }

    }
}
