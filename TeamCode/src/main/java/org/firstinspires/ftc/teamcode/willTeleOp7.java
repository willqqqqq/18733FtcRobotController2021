package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


@TeleOp(name = "willTeleOp7", group = "1")
public class willTeleOp7 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor flywheel;
    private DcMotor slide;
    private Servo wobbleLift;
    private Servo wobbleGrab;
    private Servo ringGrab;
    private Servo ringPivot;
    private Servo ringShoot;
    private BNO055IMU imu;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    Orientation angles;

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.95276;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double SPOOL_DIAMETER_INCHES = 1.825;
    static final double COUNTS_PER_INCH_SPOOL = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (SPOOL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        slide = hardwareMap.dcMotor.get("slide");
        ringGrab = hardwareMap.servo.get("ringGrab");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        wobbleLift = hardwareMap.servo.get("wobbleLift");
        wobbleGrab = hardwareMap.servo.get("wobbleGrab");
        ringShoot = hardwareMap.servo.get("ringShoot");
        ringPivot = hardwareMap.servo.get("ringPivot");
        ringGrab = hardwareMap.servo.get("ringGrab");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        boolean circleToggle1 = false;
        boolean triangleToggle1 = false;
        boolean crossToggle1 = false;
        boolean squareToggle1 = false;

        boolean circleToggle2 = false;
        boolean triangleToggle2 = false;
        boolean crossToggle2 = false;
        boolean squareToggle2 = false;
        boolean shareToggle2 = false;

        double drivePower = 1;

        wobbleLift.setPosition(.5);
        wobbleGrab.setPosition(.5);
        ringShoot.setPosition(.5);
        ringPivot.setPosition(0);
        sleep(500);
        ringGrab.setPosition(.6);

        waitForStart();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive()) {

            getHeading();

            telemetry.addData("Heading", angles.firstAngle);
            telemetry.update();

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(Math.abs(robotAngle - getHeading())) + rightX;
            final double v2 = r * Math.sin(Math.abs(robotAngle - getHeading())) - rightX;
            final double v3 = r * Math.sin(Math.abs(robotAngle - getHeading())) + rightX;
            final double v4 = r * Math.cos(Math.abs(robotAngle - getHeading())) - rightX;

            telemetry.addData("v1", v1);
            telemetry.addData("v2", v2);
            telemetry.addData("v3", v3);
            telemetry.addData("v4", v4);

            telemetry.addData("robotAngle", robotAngle);

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.update();

            frontLeft.setPower(v1 * drivePower);
            frontRight.setPower(v2 * drivePower);
            backLeft.setPower(v3 * drivePower);
            backRight.setPower(v4 * drivePower);


            if (gamepad1.cross && !crossToggle1) {
                crossToggle1 = true;
                sleep(300);
            } else if (gamepad1.cross && crossToggle1) {
                crossToggle1 = false;
                sleep(300);
            }
            if (crossToggle1) {
                drivePower = .5;
            } else if (!crossToggle1) {
                drivePower = 1;
            }


            if (gamepad1.share) {
                reset_gyro();
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.cross) {
                loadRing();
            }

            ///////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.circle && !circleToggle2) {
                circleToggle2 = true;
                sleep(300);
            } else if (gamepad2.circle && circleToggle2) {
                circleToggle2 = false;
                sleep(300);
            }
            if (circleToggle2) {
                wobbleGrab.setPosition(1);
            } else if (!circleToggle2) {
                wobbleGrab.setPosition(.5);
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.triangle && !triangleToggle2) {
                triangleToggle2 = true;
                sleep(300);
            } else if (gamepad2.triangle && triangleToggle2) {
                triangleToggle2 = false;
                sleep(300);
            }
            if (triangleToggle2) {
                flywheel.setPower(.8);
            } else if (!triangleToggle2) {
                flywheel.setPower(0);
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.right_trigger > .5) {
                ringShoot.setPosition(1);
                sleep(500);
                ringShoot.setPosition(0.5);
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.dpad_up) {
                wobbleLift.setPosition(.5);
            }

            if (gamepad2.dpad_down) {
                wobbleLift.setPosition(0);
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.share && !shareToggle2) {
                shareToggle2 = true;
                sleep(300);
            } else if (gamepad2.share && shareToggle2) {
                shareToggle2 = false;
                sleep(300);
            }
            if (shareToggle2) {
                ringGrab.setPosition(.4);
                ringPivot.setPosition(.5);
            } else if (!shareToggle2) {
                ringPivot.setPosition(0);
                ringGrab.setPosition(.6);
            }

            idle();
        }
    }

    public void loadRing () {
        int slideTarget;

        if (opModeIsActive()) {

            ringPivot.setPosition(0);
            ringGrab.setPosition(0.3);

            sleep(800);

            slideTarget = slide.getCurrentPosition() + (int) ((2.9) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.6));

            sleep(800);

            ringPivot.setPosition(1);

            sleep(800);

            ringGrab.setPosition(.5);
            sleep(800);

            ringGrab.setPosition(.3);
            sleep(800);

            ringPivot.setPosition(0);

            sleep(800);

            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideTarget = slide.getCurrentPosition() + (int) ((-2.9) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

            sleep(500);

            ringGrab.setPosition(.6);

        }
    }

    double live_gyro_value = 0;
    double gyro_offset = 0;

    double getHeading() {
        live_gyro_value = angles.firstAngle;
        return (live_gyro_value - gyro_offset);
    }

    void reset_gyro( ) {
        gyro_offset = live_gyro_value;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

