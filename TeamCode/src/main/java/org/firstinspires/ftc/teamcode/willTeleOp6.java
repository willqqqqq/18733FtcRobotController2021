package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;


@TeleOp (name = "willTeleOp6", group = "1")
public class willTeleOp6 extends BasicOpMode_Linear
//Created by Will Underhill and jackie winglke
{
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

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.95276;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double SPOOL_DIAMETER_INCHES = 1.825;
    static final double COUNTS_PER_INCH_SPOOL = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (SPOOL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        slide = hardwareMap.dcMotor.get("slide");
        ringGrab = hardwareMap.servo.get("ringGrab");

        frontRight  .setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        wobbleLift = hardwareMap.servo.get("wobbleLift");
        wobbleGrab = hardwareMap.servo.get("wobbleGrab");
        ringShoot = hardwareMap.servo.get("ringShoot");
        ringPivot = hardwareMap.servo.get("ringPivot");
        ringGrab = hardwareMap.servo.get("ringGrab");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean circleToggle1 = false;
        boolean triangleToggle1 = false;
        boolean crossToggle1 = false;
        boolean squareToggle1 = false;

        boolean circleToggle2 = false;
        boolean triangleToggle2 = false;
        boolean crossToggle2 = false;
        boolean squareToggle2 = false;

        double drivePower = 1;

        wobbleLift.setPosition(.5);
        wobbleGrab.setPosition(.5);
        ringShoot.setPosition(.5);
        ringPivot.setPosition(0);
        flywheel.setPower(.8);
        sleep(500);
        ringGrab.setPosition(.6);

        waitForStart();

        while(opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            frontLeft.setPower(v1*drivePower);
            frontRight.setPower(v2*drivePower);
            backLeft.setPower(v3*drivePower);
            backRight.setPower(v4*drivePower);

            if (gamepad1.cross && !crossToggle1){
                crossToggle1 = true;
                sleep(300);
            }
            else if (gamepad1.cross && crossToggle1){
                crossToggle1 = false;
                sleep(300);
            }
            if(crossToggle1){
                drivePower = 1;
            }
            else if(!crossToggle1){
                drivePower = .5;
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.cross){
                loadRing();
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.circle && !circleToggle2){
                circleToggle2 = true;
                sleep(300);
            }
            else if (gamepad2.circle && circleToggle2){
                circleToggle2 = false;
                sleep(300);
            }
            if(circleToggle2){
                wobbleGrab.setPosition(1);
            }
            else if(!circleToggle2){
                wobbleGrab.setPosition(.5);
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if (gamepad2.triangle && !triangleToggle2){
                triangleToggle2 = true;
                sleep(300);
            }
            else if (gamepad2.triangle && triangleToggle2){
                triangleToggle2 = false;
                sleep(300);
            }
            if(triangleToggle2){
                flywheel.setPower(.8);
            }
            else if(!triangleToggle2){
                flywheel.setPower(0);
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if(gamepad2.right_trigger > 0.2){
                ringShoot.setPosition(1);
                sleep(500);
                ringShoot.setPosition(0.5);
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            if(gamepad2.dpad_up){
                wobbleLift.setPosition(1);
            }

            if(gamepad2.dpad_down){
                wobbleLift.setPosition(0.25);
            }

            ////////////////////////////////////////
            ////////////////////////////////////////

            idle();
        }

    }
   public void loadRing() {
        int slideTarget;

        if (opModeIsActive()) {

            ringPivot.setPosition(0);
            ringGrab.setPosition(0.3);

            sleep(1000);

            slideTarget = slide.getCurrentPosition() + (int) ((3) * COUNTS_PER_INCH_SPOOL);

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

            ringPivot.setPosition(0);

            sleep(1000);

            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideTarget = slide.getCurrentPosition() + (int) ((-3) * COUNTS_PER_INCH_SPOOL);

            slide.setTargetPosition(slideTarget);

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slide.setPower(Math.abs(.4));

            sleep(500);

            ringGrab.setPosition(.6);

        }
    }
}


