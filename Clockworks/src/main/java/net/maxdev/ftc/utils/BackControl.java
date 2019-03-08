package net.maxdev.ftc.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BackControl {
    private DcMotor elevator = null; private Servo lock = null;
    private Telemetry telemetry = null;

    public void init(Telemetry oldTelemetry, HardwareMap hardwareMap) {
        telemetry = oldTelemetry;

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        lock = hardwareMap.get(Servo.class, "lock");

        lock.setDirection(Servo.Direction.FORWARD);
        lock.setPosition(1);

        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lockControl(boolean open) {
        if (open) lock.setPosition(0.75);
        else lock.setPosition(1);
    }

    public void elevatorControl(boolean goUp, double power) {
        if (goUp) {
            elevator.setTargetPosition(4200);
            elevator.setPower(power / 1.5);
        } else {
            elevator.setTargetPosition(0);
            elevator.setPower(power);
        }

        if (elevator.getTargetPosition() == 0 && elevator.getCurrentPosition() < 300 && power < 0.1)
            elevator.setPower(1);
    }

    public void elevatorOverride(int manualPosition, double power) {
        elevator.setTargetPosition(manualPosition);
        elevator.setPower(power);
    }

    public void debug() {
        telemetry.addLine()
                .addData("Elevator Current Pos", elevator.getCurrentPosition());
        telemetry.addLine()
                .addData("Lock Position", lock.getPosition());
    }
}