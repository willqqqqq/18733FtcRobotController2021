package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

@TeleOp (name = "willTeleOp6", group = "1")
public class willTeleOp6 extends BasicOpMode_Linear
{

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor flywheel;
    private DcMotor slideLeft;
    private DcMotor slideRight;
    private Servo wobbleLift;
    private Servo wobbleGrab;
    private Servo ringGrab;
    private Servo ringPivot;
    private Servo ringShoot;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.95276;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);

        wobbleLift = hardwareMap.servo.get("wobbleLift");
        wobbleGrab = hardwareMap.servo.get("wobbleGrab");
        ringShoot = hardwareMap.servo.get("ringShoot");
        ringPivot = hardwareMap.servo.get("ringPivot");

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleGrab.setPosition(1);
        flywheel.setPower(.8);

        waitForStart();

        while(opModeIsActive()) {

            boolean cross2toggle = false;
            boolean circle2toggle = false;
            boolean triangle2toggle = false;
            boolean box2toggle = false;

            int powerMult = 1;

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            frontLeft.setPower(v1/powerMult);
            frontRight.setPower(v2/powerMult);
            backLeft.setPower(v3/powerMult);
            backRight.setPower(v4/powerMult);

            /////////////////////////////////

            if(gamepad2.cross){
                loadRing(30);
            }

            if(gamepad2.circle && !circle2toggle){
                if(wobbleGrab.getPosition() == 0){
                    wobbleGrab.setPosition(.5);
                } else wobbleGrab.setPosition(0);
                circle2toggle = true;
            } else if(!gamepad1.cross) {
                circle2toggle = false;
            }



            idle();
        }

    }
    public void loadRing(double timeoutS) {
        int slideLeftTarget;
        int slideRightTarget;

        if (opModeIsActive()) {


            ringGrab.setPosition(1);


            slideLeftTarget = slideLeft.getCurrentPosition() + (int) ((2) * COUNTS_PER_INCH);
            slideRightTarget = slideRight.getCurrentPosition() + (int) ((2) * COUNTS_PER_INCH);

            slideLeft.setTargetPosition(slideLeftTarget);
            slideRight.setTargetPosition(slideRightTarget);

            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slideLeft.setPower(Math.abs(.4));
            slideRight.setPower(Math.abs(.4));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (slideLeft.isBusy() && slideRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", slideLeftTarget, slideRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        slideLeft.getCurrentPosition(),
                        slideRight.getCurrentPosition(),
                        telemetry.update());
            }
            slideLeft.setPower(0);
            slideRight.setPower(0);

            sleep(500);

            ringGrab.setPosition(0);

            sleep(500);

            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideLeftTarget = slideLeft.getCurrentPosition() + (int) ((-2) * COUNTS_PER_INCH);
            slideRightTarget = slideRight.getCurrentPosition() + (int) ((-2) * COUNTS_PER_INCH);

            slideLeft.setTargetPosition(slideLeftTarget);
            slideRight.setTargetPosition(slideRightTarget);

            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            slideLeft.setPower(Math.abs(.4));
            slideRight.setPower(Math.abs(.4));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (slideLeft.isBusy() && slideRight.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d", slideLeftTarget, slideRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        slideLeft.getCurrentPosition(),
                        slideRight.getCurrentPosition(),
                        telemetry.update());
            }

        }
    }
}


