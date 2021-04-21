package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "flywheelTesting", group = "2")
public class flywheelTesting extends LinearOpMode {

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private DcMotorEx ringShoot;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28; // This is at a 1:1 Ratio // HD Hex motor

    static final double DRIVE_GEAR_REDUCTION_SHOOTER = 20;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_ROTATION_SHOOTER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_SHOOTER);

    static final double DRIVE_GEAR_REDUCTION_FLY = 1;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_ROTATION_FLY = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_FLY);

    @Override
    public void runOpMode(){

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        ringShoot = hardwareMap.get(DcMotorEx.class, "ringShoot");

        //ringShoot.setDirection(DcMotorSimple.Direction.REVERSE );

        ringShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ringShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ringShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {

            int rps;

            rps = (int) (COUNTS_PER_ROTATION_FLY * (37));

            flywheel1.setVelocity(rps);
            flywheel2.setVelocity(rps);

            if(gamepad1.circle){
                int ringShootTarget;

                ringShootTarget = ringShoot.getCurrentPosition() + (int) (((1) * COUNTS_PER_ROTATION_SHOOTER) - 2);

                ringShoot.setTargetPosition(ringShootTarget);

                ringShoot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                runtime.reset();
                ringShoot.setPower(Math.abs(.1));

            }

        }

    }

}