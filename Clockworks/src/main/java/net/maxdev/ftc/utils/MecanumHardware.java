package net.maxdev.ftc.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumHardware {
    private DcMotor motor_bl, motor_br, motor_fl, motor_fr; // declaration of motors (wheels)
    private Servo marker; // declaration of servo for team marker
    private Telemetry telemetry; // declaration of telemetry for telemetry access (debugging purposes)

    public void init(HardwareMap hardwareMap, Telemetry oldTelemetry) {
        telemetry = oldTelemetry; // initialization of telemetry

        motor_bl = hardwareMap.get(DcMotor.class, "wheel_bl"); //
        motor_br = hardwareMap.get(DcMotor.class, "wheel_br"); //
        motor_fl = hardwareMap.get(DcMotor.class, "wheel_fl"); //
        motor_fr = hardwareMap.get(DcMotor.class, "wheel_fr"); // initialization of motors

        marker = hardwareMap.get(Servo.class, "marker"); // initialization of servo for marker

        motor_bl.setDirection(DcMotorSimple.Direction.REVERSE); //
        motor_br.setDirection(DcMotorSimple.Direction.FORWARD); //
        motor_fl.setDirection(DcMotorSimple.Direction.REVERSE); //
        motor_fr.setDirection(DcMotorSimple.Direction.FORWARD); // tell the motors which way to go

        motor_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //
        motor_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //
        motor_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //
        motor_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // tell the motors to reset the encoders

        marker.setDirection(Servo.Direction.FORWARD); //
        marker.setPosition(1); // set the direction and position for the marker servo

        motor_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // bypass Rev Expansion Bug that makes the
        motor_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // encoders go haywire if they are reset
        motor_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // then attempted to read from, while within
        motor_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // RUN_USING_ENCODER mode

        motor_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //
        motor_br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // if the power of the motors is
        motor_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set to 0, the motors will try
        motor_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // to keep their current position
    }

    private void normalizePower(double[] power) {
        power[0] = Range.clip(power[0], -1, 1); power[1] = Range.clip(power[1], -1, 1);
        power[2] = Range.clip(power[2], -1, 1); power[3] = Range.clip(power[3], -1, 1);
        // limitation of power so the battery doesn't die/drain from over-voltage requirements from hardware
    }

    public void drive(double x, double y, double rotation) {
        double[] power = new double[4]; // declaration of power map

        power[0] = y + x + rotation; power[1] = y - x - rotation; //
        power[2] = y - x + rotation; power[3] = y + x - rotation; // math needed for mecanum control

        normalizePower(power); // enforce power limits

        motor_bl.setPower(power[0]); motor_br.setPower(power[1]);
        motor_fl.setPower(power[2]); motor_fr.setPower(power[3]);
    }
}
