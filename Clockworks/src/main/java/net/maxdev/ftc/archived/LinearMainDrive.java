package net.maxdev.ftc.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.maxdev.ftc.archived.utils.BackControl;
import net.maxdev.ftc.archived.utils.DogeGoldVision;
import net.maxdev.ftc.archived.utils.FrontControl;
import net.maxdev.ftc.archived.utils.MecanumWheels;

@Disabled
@TeleOp(name = "LinearMainDrive", group = "debug")
public class LinearMainDrive extends LinearOpMode {
    private MecanumWheels wheels = new MecanumWheels();
    private FrontControl front = new FrontControl();
    private BackControl back = new BackControl();
    private DogeGoldVision detector = new DogeGoldVision();

    private ElapsedTime runtime = new ElapsedTime();
    private int positionSet = 0;

    @Override
    public void runOpMode() {
        wheels.init(telemetry, hardwareMap);
        front.init(hardwareMap, telemetry);
        back.init(telemetry, hardwareMap);
        detector.init(hardwareMap, telemetry);

        waitForStart();
        detector.enable(true);

        while (opModeIsActive()) {
            wheels.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper);
            if (gamepad2.left_bumper) front.broom_control(-gamepad2.left_trigger, gamepad2.right_trigger);
            else front.broom_control(gamepad2.left_trigger, gamepad2.right_trigger);

            if (gamepad2.dpad_down && runtime.seconds() > 0.75 && positionSet < 4) {
                positionSet++; front.slide_position(positionSet); runtime.reset();
            } else if (gamepad2.dpad_up && runtime.seconds() > 0.75 && positionSet > 0) {
                positionSet--; front.slide_position(positionSet); runtime.reset();
            }
            front.slide_extension(gamepad2.a, gamepad2.left_stick_y);
            back.elevatorControl(gamepad1.left_bumper, gamepad1.left_trigger);

            detector.debug();
            front.debug();
            back.debug();
            telemetry.update();
        }

        detector.enable(false);
    }
}
