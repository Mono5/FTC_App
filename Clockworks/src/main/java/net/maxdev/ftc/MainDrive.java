package net.maxdev.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.maxdev.ftc.utils.BackControl;
import net.maxdev.ftc.utils.DogeGoldVision;
import net.maxdev.ftc.utils.FrontControl;
import net.maxdev.ftc.utils.MecanumWheels;

@TeleOp(name = "MainDrive", group = "Drive")
public class MainDrive extends OpMode {
    private MecanumWheels wheels = new MecanumWheels();
    private FrontControl front = new FrontControl();
    private BackControl back = new BackControl();

    private ElapsedTime runtime = new ElapsedTime();
    private int positionSet = 0;

    @Override
    public void init() {
        wheels.init(telemetry, hardwareMap);
        front.init(hardwareMap, telemetry);
        back.init(telemetry, hardwareMap);
    }
    @Override
    public void init_loop() {}
    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        wheels.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper);
        if (gamepad2.left_bumper) front.broom_control(-gamepad2.left_trigger, gamepad2.right_trigger);
        else front.broom_control(gamepad2.left_trigger, gamepad2.right_trigger);

        if (gamepad2.dpad_down && runtime.seconds() > 0.75 && positionSet < 5) {
            positionSet++; front.slide_position(positionSet); runtime.reset();
        } else if (gamepad2.dpad_up && runtime.seconds() > 0.75 && positionSet > 0) {
            positionSet--; front.slide_position(positionSet); runtime.reset();
        }
        front.slide_extension(gamepad2.a, gamepad2.left_stick_y);
        back.elevatorControl(gamepad1.left_bumper, gamepad1.left_trigger);

        front.debug();
        back.debug();
        telemetry.update();
    }
    @Override
    public void stop() {}
}