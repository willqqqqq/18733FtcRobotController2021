package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;


@Autonomous(name = "intakeTesting", group = "2")
public class intakeTesting  extends BasicOpMode_Linear {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor slide;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private DcMotorEx ringShoot;
    private Servo wobbleLift;
    private Servo wobbleGrab;
    private Servo ringGrab;
    private Servo ringPivot;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.95276;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double SPOOL_DIAMETER_INCHES = 1.825;
    static final double COUNTS_PER_INCH_SPOOL = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (SPOOL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_GEAR_REDUCTION_SHOOTER = 20;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_ROTATION_SHOOTER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_SHOOTER);

    static final double DRIVE_GEAR_REDUCTION_FLY = 1;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_ROTATION_FLY = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_FLY);

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        slide = hardwareMap.dcMotor.get("slide");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        ringShoot = hardwareMap.get(DcMotorEx.class, "ringShoot");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        wobbleLift = hardwareMap.servo.get("wobbleLift");
        wobbleGrab = hardwareMap.servo.get("wobbleGrab");
        ringPivot = hardwareMap.servo.get("ringPivot");
        ringGrab = hardwareMap.servo.get("ringGrab");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ringShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ringShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ringShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ringPivot.setPosition(0);
        sleep(500);
        ringGrab.setPosition(.6);

        waitForStart();

        while (opModeIsActive()) {

            stackGrab();
            sleep(5000);
            stop();
        }

    }

    public void stackGrab() {

        int slideTarget;

        if (opModeIsActive()) {

            slideTarget = slide.getCurrentPosition() + (int) ((2.1) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

            sleep(1000);

            ringPivot.setPosition(0.15);
            ringGrab.setPosition(0.3);

            sleep(1000);

            slideTarget = slide.getCurrentPosition() + (int) ((2.5) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

            sleep(1000);

            ringPivot.setPosition(1);

            sleep(1000);

            ringGrab.setPosition(.5);
            sleep(1000);

            ringGrab.setPosition(.3);
            sleep(1000);

            ringPivot.setPosition(0.15);
            sleep(1000);

            ringGrab.setPosition(.6);
            sleep(1000);

            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideTarget = slide.getCurrentPosition() + (int) ((-2.8) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

            sleep(1000);

            ringPivot.setPosition(0);
            ringGrab.setPosition(0.15);

            sleep(1000);

            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideTarget = slide.getCurrentPosition() + (int) ((2.7) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

            sleep(1000);

            ringPivot.setPosition(1);

            sleep(1000);

            ringGrab.setPosition(.5);
            sleep(1000);

            ringGrab.setPosition(.3);
            sleep(1000);

            ringPivot.setPosition(0.15);
            sleep(1000);

            ringGrab.setPosition(.6);
            sleep(1000);


            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideTarget = slide.getCurrentPosition() + (int) ((-3.5) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

            sleep(1000);

            ringPivot.setPosition(0.15);
            ringGrab.setPosition(0.3);

            sleep(1000);

            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideTarget = slide.getCurrentPosition() + (int) ((3.9) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

            sleep(1000);

            ringPivot.setPosition(1);

            sleep(1000);

            ringGrab.setPosition(.5);
            sleep(1000);

            ringGrab.setPosition(.3);
            sleep(1000);

            ringPivot.setPosition(0.15);
            sleep(1000);

            ringGrab.setPosition(.6);
            sleep(1000);

            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideTarget = slide.getCurrentPosition() + (int) ((-4.35) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

        }

    }

}
