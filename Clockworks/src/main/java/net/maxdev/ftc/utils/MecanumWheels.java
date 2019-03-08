package net.maxdev.ftc.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

public class MecanumWheels {
    private DcMotor motor_bl = null; private DcMotor motor_br = null;
    private DcMotor motor_fl = null; private DcMotor motor_fr = null;
    private static double countsPerInch = 1120 / (4 * Math.PI);
    private static double driveCoef = 0.06;
    private static int driveThreshold = (int) (0.2 * countsPerInch);
    private BNO055IMU imu = null;
    private double headingResetValue;
    private Servo marker = null;

    private Telemetry telemetry = null; private HardwareMap hardwareMap = null;

    public void init(Telemetry oldTelemetry, HardwareMap oldHardwareMap) {
        telemetry = oldTelemetry; hardwareMap = oldHardwareMap;

        motor_bl = hardwareMap.get(DcMotor.class, "wheel_bl");
        motor_br = hardwareMap.get(DcMotor.class, "wheel_br");
        motor_fl = hardwareMap.get(DcMotor.class, "wheel_fl");
        motor_fr = hardwareMap.get(DcMotor.class, "wheel_fr");
        marker = hardwareMap.get(Servo.class, "marker");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motor_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_bl.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_br.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_fl.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_fr.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        marker.setDirection(Servo.Direction.FORWARD);
        marker.setPosition(1);
        headingResetValue = getAbsoluteHeading();
    }

    public void drive(double x, double y, double rotation, boolean halfSpeed) {
        double[] power = new double[4];

        power[0] = y + x - rotation; //back left
        power[1] = y - x + rotation; //back right
        power[2] = y - x - rotation; //front left
        power[3] = y + x + rotation; //front right

        normalizeMecanum(power);

        if (halfSpeed) {
            power[0] = power[0] / 2;
            power[1] = power[1] / 2;
            power[2] = power[2] / 2;
            power[3] = power[3] / 2;
        }

        motor_bl.setPower(power[0]);
        motor_br.setPower(power[1]);
        motor_fl.setPower(power[2]);
        motor_fr.setPower(power[3]);
    }

    private void normalizeMecanum(double[] power) {
        power[0] = Math.max(Math.min(power[0], 1), -1);
        power[1] = Math.max(Math.min(power[1], 1), -1);
        power[2] = Math.max(Math.min(power[2], 1), -1);
        power[3] = Math.max(Math.min(power[3], 1), -1);
    }
    public void encoderDrive(double backLeftInches, double backRightInches, double frontLeftInches,
                             double frontRightInches, double maxSpeed, double timeoutInSeconds, LinearOpMode opMode) {
        double a, b, c, d;
        double[] speed = new double[4];
        double[] error = new double[4];
        ElapsedTime runtime = new ElapsedTime();

        a = motor_fr.getCurrentPosition() + (int) (frontRightInches * countsPerInch);
        b = motor_fl.getCurrentPosition() + (int) (frontLeftInches * countsPerInch);
        c = motor_br.getCurrentPosition() + (int) (backRightInches * countsPerInch);
        d = motor_bl.getCurrentPosition() + (int) (backLeftInches * countsPerInch);

        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < timeoutInSeconds && (Math.abs(motor_fr.getCurrentPosition() - a) >= driveThreshold || Math.abs(motor_fl.getCurrentPosition() - b) >= driveThreshold
                || Math.abs(motor_br.getCurrentPosition() - c) >= driveThreshold || Math.abs(motor_bl.getCurrentPosition() - d) >= driveThreshold)) {
            error[0] = a - motor_fr.getCurrentPosition();
            speed[0] = Range.clip(error[0] * driveCoef, -maxSpeed, maxSpeed);
            error[1] = b - motor_fl.getCurrentPosition();
            speed[1] = Range.clip(error[1] * driveCoef, -maxSpeed, maxSpeed);
            error[2] = c - motor_br.getCurrentPosition();
            speed[2] = Range.clip(error[2] * driveCoef, -maxSpeed, maxSpeed);
            error[3] = d - motor_bl.getCurrentPosition();
            speed[3] = Range.clip(error[3] * driveCoef, -maxSpeed, maxSpeed);

            motor_fr.setPower(speed[0]);
            motor_fl.setPower(speed[1]);
            motor_br.setPower(speed[2]);
            motor_bl.setPower(speed[3]);
        }

        motor_fr.setPower(0);
        motor_fl.setPower(0);
        motor_br.setPower(0);
        motor_bl.setPower(0);
    }

    private double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    private double getRelativeHeading() {
        return getAbsoluteHeading() - headingResetValue;
    }
    private int gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        int correctCount = 0;
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0;
        if (delta > 180.0) delta -= 360.0;
        if (Math.abs(delta) > gyroRange) {
            correctCount = 0;
            double gyroMod = delta / 45.0;
            if (Math.abs(gyroMod) > 1.0) gyroMod = Math.signum(gyroMod);
            turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else {
            correctCount++;
            turn(0.0);
        }
        return correctCount;
    }
    private void turn(double sPower) {
        motor_fl.setPower(-sPower); motor_bl.setPower(-sPower);
        motor_fr.setPower(sPower); motor_br.setPower(sPower);
    }

    public void rotate(double targetDegrees, double maxSpeed, double error, double timeoutInSeconds, LinearOpMode opMode) {
        double heading = getRelativeHeading();
        ElapsedTime runtime = new ElapsedTime();

        runtime.reset();
        while (gyroCorrect(targetDegrees, error, heading, 0.1, maxSpeed - 0.1) == 0
                && runtime.seconds() < timeoutInSeconds && opMode.opModeIsActive()) {
            heading = getRelativeHeading();
            gyroCorrect(targetDegrees, error, heading, 0.1, maxSpeed - 0.1);
        }
        runtime.reset(); while (runtime.seconds() < 0.5 && opMode.opModeIsActive()) {}
        while (gyroCorrect(targetDegrees, error, heading, 0.1, maxSpeed - 0.1) == 0
                && runtime.seconds() < timeoutInSeconds / 2 && opMode.opModeIsActive()) {
            heading = getRelativeHeading();
            gyroCorrect(targetDegrees, error, heading, 0.1, maxSpeed - 0.1); //
        }
    }
}
