package net.maxdev.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.maxdev.ftc.utils.BackControl;
import net.maxdev.ftc.utils.DogeGoldVision;
import net.maxdev.ftc.utils.MecanumWheels;

@Autonomous(name = "AutoLeft", group = "Autonomous")
public class AutoLeft extends LinearOpMode {
    private MecanumWheels wheels = null;
    private BackControl back = null;
    private DogeGoldVision detector = null;

    @Override
    public void runOpMode() {
        boolean goldStatus = false;
        wheels = new MecanumWheels();
        wheels.init(telemetry, hardwareMap);

        back = new BackControl();
        back.init(telemetry, hardwareMap);

        detector = new DogeGoldVision();
        detector.init(hardwareMap, telemetry);

        waitForStart();
        detector.enable(true);
        //back.elevatorOverride(-75, 1);
        sleep(750);
        //back.lockControl(true);
        //sleep(500);

        //back.elevatorControl(true, 0.8);
        //sleep(4000);

        wheels.timeDrive(0.5, 1, 1, 1, 1, 0.3);
        //back.elevatorControl(false, 0.6);
        sleep(1000);
        if (detector.getLocation() < 330 && detector.getLocation() > 250) {
            detector.enable(false);
            wheels.timeDrive(0.5, 1, 1, 1, 1, 0.5);
            goldStatus = true;
        } else {
            wheels.timeDrive(0.5, 1, -1, -1, 1, 0.35);
            sleep(1000)
            if (detector.getLocation() < 330 && detector.getLocation() > 250) {
                detector.enable(false);
                wheels.timeDrive(0.5, 1, 1, 1, 1, 0.5);
                goldStatus = true;
            } else {
                detector.enable(false);
                wheels.timeDrive(1, -1, 1, 1, -1, 0.35);
                wheels.timeDrive(0.5, 1, 1, 1, 1, 0.5);
            }
        }
    }
}
