package net.maxdev.ftc.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class MecanumHardware {
    private DcMotor motor_bl, motor_br, motor_fl, motor_fr; // declaration of motors (wheels)
    private Servo marker; // declaration of servo for team marker
    private Telemetry telemetry; // declaration of telemetry for telemetry access (debugging purposes)

    private BNO055IMU imu = null; //
    private double headingResetValue; // declaration of Rev Exp Hub gyro and reset value of front degrees

    private static final double DRIVE_GEAR_REDUCTION = 1; //
    private static final double COUNTS_PER_INCH = (1120 * DRIVE_GEAR_REDUCTION) / (4 * Math.PI); //
    private static final int DRIVE_THRESHOLD = (int) (0.2 * COUNTS_PER_INCH); //
    private static final double P_DRIVE_COEFF = 0.013; // values for easier Autonomous control

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

        // here begins the initialization and calibration of the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // here ends the initialization and calibration of the gyro
    }

    private void normalizePower(double[] power) {
        power[0] = Range.clip(power[0], -1, 1); power[1] = Range.clip(power[1], -1, 1);
        power[2] = Range.clip(power[2], -1, 1); power[3] = Range.clip(power[3], -1, 1);
        // limitation of power so the battery doesn't die/drain from over-voltage requirements from hardware
    }

    public void drive(double x, double y, double rotation) {
        double[] power = new double[4]; // declaration of power map

        power[0] = y + x + rotation; power[1] = y - x - rotation; // initialization of power map
        power[2] = y - x + rotation; power[3] = y + x - rotation; // + math needed for mecanum control

        normalizePower(power); // enforce power limits

        motor_bl.setPower(power[0]); motor_br.setPower(power[1]); //
        motor_fl.setPower(power[2]); motor_fr.setPower(power[3]); // apply power map to motors
    }

    public void encoderDrive(LinearOpMode opMode, double backLeftInches, double backRightInches, double frontLeftInches, double frontRightInches, double maxSpeed) {
        double a, b, c, d; //
        double[] speed = new double[4]; //
        double[] error = new double[4]; // declaration of mandatory values

        a = (int) (frontRightInches * COUNTS_PER_INCH); b = (int) (frontLeftInches * COUNTS_PER_INCH); //
        c = (int) (backRightInches * COUNTS_PER_INCH); d = (int) (backLeftInches * COUNTS_PER_INCH); // initialization of target values

        while (opMode.opModeIsActive() && (Math.abs(motor_fr.getCurrentPosition() - a) >= DRIVE_THRESHOLD || Math.abs(motor_fl.getCurrentPosition() - b) >= DRIVE_THRESHOLD
                || Math.abs(motor_br.getCurrentPosition() - c) >= DRIVE_THRESHOLD || Math.abs(motor_bl.getCurrentPosition() - d) >= DRIVE_THRESHOLD)) {
            error[0] = a + motor_fr.getCurrentPosition();
            speed[0] = Range.clip(error[0] * P_DRIVE_COEFF, -maxSpeed, maxSpeed);
            error[1] = b + motor_fl.getCurrentPosition();
            speed[1] = Range.clip(error[1] * P_DRIVE_COEFF, -maxSpeed, maxSpeed);
            error[2] = c + motor_br.getCurrentPosition();
            speed[2] = Range.clip(error[2] * P_DRIVE_COEFF, -maxSpeed, maxSpeed);
            error[3] = d + motor_bl.getCurrentPosition();
            speed[3] = Range.clip(error[3] * P_DRIVE_COEFF, -maxSpeed, maxSpeed);

            motor_fr.setPower(speed[0]);
            motor_fl.setPower(speed[1]);
            motor_br.setPower(speed[2]);
            motor_bl.setPower(speed[3]);
        } // this while is to work as RUN_TO_POSITION, but including a ramp-up of the motors and a much efficient method setting powers while moving

        motor_fr.setPower(0); //
        motor_fl.setPower(0); //
        motor_br.setPower(0); //
        motor_bl.setPower(0); // this fully stops the motors when we reach the desired positions / distance
    }

    public void marker (boolean drop) { // function for setting the position of the marker servo in drop or hold
        if (drop) marker.setPosition(0.5);
        else marker.setPosition(1);
    }

    private double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        // as the name implies, this calls the default get heading function in a much cuter way
    }
    private double getRelativeHeading() {
        return getAbsoluteHeading() - headingResetValue;
        // same thing as above, but loading an error range too
    }

    private int gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        int correctCount = 0;
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //
        if (delta > 180.0) delta -= 360.0; // this modifies the value to limit it in the valid range for the gyro
        if (Math.abs(delta) > gyroRange) { //checks if the value is higher than the error range given
            double gyroMod = delta / 45.0; // corrects 45 degrees turns
            if (Math.abs(gyroMod) > 1.0) gyroMod = Math.signum(gyroMod); // set the motors at the specific power + a ramp-up for turning
            turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else { // this else stops the re-run of the program if the target is already achieved
            correctCount++;
            turn(0.0);
        }
        return correctCount;
    }
    private void turn(double sPower) {
        motor_fl.setPower(-sPower); motor_bl.setPower(-sPower); // set the power of the motors based on the gyro reading
        motor_fr.setPower(sPower); motor_br.setPower(sPower); // and parameters given in rotate()
    }

    public void rotate(LinearOpMode opMode, double targetDegrees, double maxSpeed, double error, double timeoutInSeconds) {
        double heading = getRelativeHeading(); // declare and initialize current value of direction
        ElapsedTime runtime = new ElapsedTime(); // declare and initialize timer for timeouts

        runtime.reset(); // reset the timer for timeouts to be in effect
        while (gyroCorrect(targetDegrees, error, heading, 0.1, maxSpeed - 0.1) == 0 // first check if the gyro
                && runtime.seconds() < timeoutInSeconds && opMode.opModeIsActive()) { // is aligned, and if not it will get aligned
            heading = getRelativeHeading();
            gyroCorrect(targetDegrees, error, heading, 0.1, maxSpeed - 0.1);
        }
        runtime.reset(); // prepare the timer for the second check
        while (gyroCorrect(targetDegrees, error, heading, 0.1, maxSpeed - 0.1) == 0 // runs the second check of the gyro
                && runtime.seconds() < timeoutInSeconds / 2 && opMode.opModeIsActive()) { // just in case the robot went over the desired position
            heading = getRelativeHeading();
            gyroCorrect(targetDegrees, error, heading, 0.1, maxSpeed - 0.1); //
        }
    }
}
