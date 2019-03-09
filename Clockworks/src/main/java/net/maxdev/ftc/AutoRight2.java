package net.maxdev.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.maxdev.ftc.utils.DogeGoldVision;
import net.maxdev.ftc.utils.MecanumWheels;

@Autonomous(name = "AutoRight", group = "Autonomous")
public class AutoRight2 extends LinearOpMode {
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
        if (detector.getLocation() > 200 && detector.getLocation() < 400) {
            detector.enable(false);
            wheels.encoderDrive(50, 50, 50, 50, 0.8, 5);
            wheels.rotate(315, 0.7, 1, 3);
            wheels.marker(true);
            wheels.rotate(225, 0.8, 1, 3);
            wheels.rotate(125, 0.8, 1, 5);
            wheels.encoderDrive(60, 60, 60, 60, 1, 5);
        } else {
            wheels.rotate(30, 0.7, 1, 3);
            if (detector.getLocation() > 200 && detector.getLocation() < 400) {
                detector.enable(false);
                wheels.encoderDrive(35, 35, 35, 35, 0.6, 4);
                wheels.rotate(210, 0.9, 1, 5);
                wheels.encoderDrive(24, -24, -24, 24, 0.9, 3);
                wheels.marker(true);
                wheels.encoderDrive(-50, 50, 50, -50, 0.9, 6);
            } else {
                detector.enable(false);
                wheels.rotate(320, 0.7, 2, 4);
                wheels.encoderDrive(24, 24, 24, 24, 0.9, 3);
                wheels.rotate(0, 0.8, 1, 3);
                wheels.encoderDrive(27, 27, 27, 27, 0.9, 3);
                wheels.rotate(315, 0.8, 1, 3);
                wheels.marker(true);
                //wheels.rotate(225, 0.9, 1, 3);
            }
        }
    }
}