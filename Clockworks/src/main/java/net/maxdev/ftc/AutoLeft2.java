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
        detector = new DogeGoldVision();
        wheels.init(telemetry, hardwareMap);
        detector.init(hardwareMap, telemetry);

        waitForStart();
        detector.enable(true);
        wheels.encoderDrive(8, 8, 8, 8, 0.6, 2);
        sleep(750);
        if (detector.getLocation() > 200 && detector.getLocation() < 400) {
            detector.enable(false);
            wheels.encoderDrive(12, 12, 12, 12, 0.6, 2);
        } else {
            wheels.encoderDrive(24, -24, -24, 24, 0.8, 3);
            sleep(1000);
            if (detector.getLocation() > 200 && detector.getLocation() < 400) {
                detector.enable(false);
                wheels.encoderDrive(12, 12, 12, 12, 0.6, 2);
            } else {
                detector.enable(false);
                wheels.encoderDrive(-48, 48, 48, -48, 0.8, 4);
                wheels.encoderDrive(12, 12, 12, 12, 0.6, 2);
            }
        }
    }
}
