package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Omega_TeleOp_Shayaan extends LinearOpMode{

    private DcMotorEx leftFront, leftRear, rightRear, rightFront,Intake,Channel,HangL,HangR;
    private Servo Drone,HangReleaseR,HangReleaseL,GrabFrontL,GrabFrontR;

    private CRServo LeadScrew;

    @Override
    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        Drone = hardwareMap.get(Servo.class, "Drone");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        HangReleaseR = hardwareMap.get(Servo.class, "HangReleaseR");
        HangReleaseL = hardwareMap.get(Servo.class, "HangReleaseL");
        HangL = hardwareMap.get(DcMotorEx.class, "HangL");
        HangR = hardwareMap.get(DcMotorEx.class, "HangR");
        Channel = hardwareMap.get(DcMotorEx.class, "Channel");
        GrabFrontL = hardwareMap.get(Servo.class, "GrabFrontL");
        GrabFrontR = hardwareMap.get(Servo.class, "GrabFrontR");
        LeadScrew = hardwareMap.get(CRServo.class, "LeadScrew");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        HangL.setDirection(DcMotorSimple.Direction.REVERSE);
        int f = 0,a=0;

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Channel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()){
            double x = gamepad1.left_stick_x*0.5;
            double y = gamepad1.left_stick_y*0.5;
            double z = gamepad1.right_stick_x*0.5;
            double LFP =  x+y+z;
            double LBP = x+y-z;
            double RFP = x-y-z;
            double RBP = x-y+z;
            leftFront.setPower(LFP);rightFront.setPower(RFP);leftRear.setPower(LBP);rightRear.setPower(RBP);

            //Channel Motor
            if (gamepad1.dpad_down){
                Channel.setPower(-0.4);
            } else if (gamepad1.dpad_up){
                Channel.setPower(0.4);
            } else {
                Channel.setPower(0);
            }

            //LeadScrew Servo
            if (gamepad1.dpad_left){
                LeadScrew.setPower(0.5);
            }
            else if (gamepad1.dpad_right){
                LeadScrew.setPower(-0.5);
            }
            else
            {
                LeadScrew.setPower(0);
            }

            //Hang Motors
             if (gamepad1.right_bumper)
            {
                HangL.setPower(0.4);
                HangR.setPower(0.4);
            }
            else if (gamepad1.left_bumper)
            {
                HangR.setPower(-0.4);
                HangL.setPower(-0.4);
            }
            else {
                HangR.setPower(0);
                HangL.setPower(0);
            }
//            if (gamepad1.triangle){
//                if(a==1)
//                {
//                    Drone.setPosition(1);
//                    a = 0;
//                }
//                Drone.setPosition(0);
//                a = 1;
//            }
            if(gamepad1.cross)
            {
                if(f==1)
                {
                    Intake.setPower(0);
                    f = 0;
                }
                else {
                    f = 1;
                    Intake.setPower(0.5);
                }
            }
//            if(gamepad1.square)
//            {
//                GrabFrontL.setPosition(1);
//                GrabFrontR.setPosition(1);
//            }
//            else if(gamepad1.circle)
//            {
//                GrabFrontL.setPosition(0);
//                GrabFrontR.setPosition(0);
//            }
            if(gamepad1.square)
            {
                HangReleaseL.setPosition(1);
                HangReleaseR.setPosition(0);
            }
        }
    }

}
