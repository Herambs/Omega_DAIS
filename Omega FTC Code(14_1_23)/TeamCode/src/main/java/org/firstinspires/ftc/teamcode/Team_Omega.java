package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Team_Omega extends OpMode {

    private DcMotor LF,LB,RB,RF;

    private DcMotor intake, H1,H2, channel;
    private Servo boxLeft, boxRight;

    private Servo Drone;

    private Servo InnerGrab;


    @Override
    public void init() {

        LF=hardwareMap.get(DcMotorEx.class,"LF");
        LB=hardwareMap.get(DcMotorEx.class,"LB");
        RB=hardwareMap.get(DcMotorEx.class,"RB");
        RF=hardwareMap.get(DcMotorEx.class,"RF");
        intake=hardwareMap.get(DcMotorEx.class,"intake");
        H1=hardwareMap.get(DcMotorEx.class,"H1");
        H2=hardwareMap.get(DcMotorEx.class,"H2");
        channel=hardwareMap.get(DcMotorEx.class,"channel");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        H1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        H2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        channel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        boxLeft= hardwareMap.get(Servo.class,"boxLeft");
        boxRight= hardwareMap.get(Servo.class,"boxRight");
        InnerGrab= hardwareMap.get(Servo.class,"innerGrab");
//        S3= hardwareMap.get(Servo.class,"S3");
//        S4= hardwareMap.get(Servo.class,"S4");
//        LeadScrew= hardwareMap.get(CRServo.class,"S5");

        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);

        boxLeft.setPosition(0);
        boxRight.setPosition(1);
        InnerGrab.setPosition(0.5);

    }

    @Override
    public void loop() {

        double A= gamepad1.left_stick_y*0.6;
        double B= gamepad1.left_stick_x*0.6;
        double C= gamepad1.right_stick_x*0.6;

        double LFP=A-C-B;
        double LBP=A-C+B;
        double RBP=A+C-B;
        double RFP=A+C+B;

        LF.setPower(LFP);
        LB.setPower(LBP);
        RB.setPower(RBP);
        RF.setPower(RFP);

        //Extension Channel
        if(gamepad1.dpad_up){
            channel.setPower(0.8);
        }else if(gamepad1.dpad_down){
            channel.setPower(-0.4);
        }else{
            channel.setPower(0);
        }

        // Intake Motor with Same button
        if(gamepad1.triangle) {
            intake.setPower(0.65);
        }else if(gamepad1.cross){
            intake.setPower(0);
        }

        if(gamepad1.right_trigger!=0){
            H1.setPower(0.5);
            H2.setPower(-0.5);
        }else if(gamepad1.left_trigger!=0){
            H1.setPower(-0.8);
            H2.setPower(0.8);
        }else{
            H1.setPower(-0.05);
            H2.setPower(0.05);
        }

        if(gamepad1.circle){
            boxLeft.setPosition(0.7);
            boxRight.setPosition(0.3);
        }else if(gamepad1.square){
            boxLeft.setPosition(0);
            boxRight.setPosition(1);
        }

        if(gamepad1.dpad_right){
            InnerGrab.setPosition(0.2);
        }else if(gamepad1.dpad_left){
            InnerGrab.setPosition(0.8);
        }else if(gamepad1.left_bumper){
            InnerGrab.setPosition(0.5);
        }
    }
}
