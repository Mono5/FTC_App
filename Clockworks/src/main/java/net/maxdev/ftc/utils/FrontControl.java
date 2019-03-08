package net.maxdev.ftc.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FrontControl {
    private DcMotor broom = null; private DcMotor slide_d = null;
    private DcMotor slide_e = null; private Servo broom_dir = null;
    private HardwareMap hardwareMap = null; private Telemetry telemetry = null;
    private int oldPosition = 0;

    public void init(HardwareMap oldHardwareMap, Telemetry oldTelemetry) {
        hardwareMap = oldHardwareMap; telemetry = oldTelemetry;

        broom = hardwareMap.get(DcMotor.class, "broom");
        slide_d = hardwareMap.get(DcMotor.class, "slide_direction");
        slide_e = hardwareMap.get(DcMotor.class, "slide_extension");
        broom_dir = hardwareMap.get(Servo.class, "broom_dir");

        broom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        broom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        broom.setDirection(DcMotorSimple.Direction.REVERSE);

        slide_d.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slide_e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_e.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_e.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        broom_dir.setDirection(Servo.Direction.FORWARD);
        broom_dir.setPosition(0);
    }

    public void broom_control(double broomPower, double broomPosition) {
        if (broomPosition < 0) broomPosition = 0;
        else if (broomPosition > 1) broomPosition = 1;
        if (broomPower < -1) broomPower = -1;
        else if (broomPower > 1) broomPower = 1;

        broom_dir.setPosition(broomPosition);
        broom.setPower(broomPower);
    }

    public void slide_position(int position) {
        if (position == 0) {
            slide_d.setTargetPosition(0);
            slide_d.setPower(0.4);
        } else if (position == 1) {
            slide_d.setTargetPosition(-220);
            slide_d.setPower(0.8);
        } else if (position == 2) {
            slide_d.setTargetPosition(-400);
            slide_d.setPower(0.5);
        } else if (position == 3) {
            slide_d.setTargetPosition(-570);
            slide_d.setPower(0.7);
        } else if (position == 4) {
            slide_d.setTargetPosition(-800);
            slide_d.setPower(0.8);
        } else if (position == 5) {
            slide_d.setTargetPosition(-1100);
            slide_d.setPower(0.4);
        }
        if (position <= 5 && position >= 0) oldPosition = position;
    }

    public void slide_extension(boolean free, double power) {
        if (slide_e.getCurrentPosition() < -50 && power < 0) {
            slide_e.setPower(0);
            return;
        } else if (slide_e.getCurrentPosition() > 650 && power > 0) {
            slide_e.setPower(0);
            return;
        }

        if (free) slide_e.setPower(-power);
        else slide_e.setPower(power);
    }

    public void debug() {
        telemetry.addLine()
                .addData("Slide_D Encoder", slide_d.getCurrentPosition())
                .addData("Slide_E Encoder", slide_e.getCurrentPosition());
        telemetry.addLine()
                .addData("Broom Power", broom.getPower())
                .addData("Broom Direction", broom_dir.getPosition());
        telemetry.addLine()
                .addData("Slide_D Power", slide_d.getPower())
                .addData("Slide_D Target", slide_d.getTargetPosition())
                .addData("Slide_D PosVal", oldPosition);
    }
}