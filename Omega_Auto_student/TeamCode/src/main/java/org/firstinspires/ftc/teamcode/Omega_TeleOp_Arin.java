package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Omega_TeleOp_Arin extends OpMode {

    private DcMotor LF, LB, RB, RF;

    private DcMotor intake, H1, H2, channel;
    private Servo S0, S1, S2, S3, S4;

    private CRServo S5;

    boolean flag = true;


    @Override
    public void init() {

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        H1 = hardwareMap.get(DcMotorEx.class, "H1");
        H2 = hardwareMap.get(DcMotorEx.class, "H2");
        channel = hardwareMap.get(DcMotorEx.class, "channel");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        H1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        H2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        channel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        S0 = hardwareMap.get(Servo.class, "HangReleaseR");
//        S1 = hardwareMap.get(Servo.class, "HangReleaseL");
        S2 = hardwareMap.get(Servo.class, "S2"); // right placing servo (0 to 1)
        S3 = hardwareMap.get(Servo.class, "S3"); // left placing servo (1 to 0)
//        S4 = hardwareMap.get(Servo.class, "S4");
//        S5 = hardwareMap.get(CRServo.class, "LeadScrew");

        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);

        S2.setPosition(0);
        S3.setPosition(1);

    }

    @Override
    public void loop() {

        double A = gamepad1.left_stick_y*0.5;
        double B = gamepad1.left_stick_x*0.5;
        double C = gamepad1.right_stick_x*0.5;

        double LFP = A + C + B;
        double LBP = A + C - B;
        double RBP = A - C + B;
        double RFP = A - C - B;

        LF.setPower(LFP);
        LB.setPower(LBP);
        RB.setPower(RBP);
        RF.setPower(RFP);

        //Extension Channel
        if (gamepad1.dpad_up) {
            channel.setPower(0.6);
        } else if (gamepad1.dpad_down) {
            channel.setPower(-0.5);
        } else {
            channel.setPower(0);
        }

        // Lead Screw Servo
//        if (gamepad1.dpad_right) {
//            S5.setPower(0.6);
//        } else if (gamepad1.dpad_left) {
//            S5.setPower(0.6);
//        } else {
//            S5.setPower(0);
//        }

        // Intake Motor
        if (gamepad1.triangle) {
            intake.setPower(0.6);
        }
        if (gamepad1.cross) {
            intake.setPower(0);
        }

        //Intake
        if (gamepad1.right_trigger != 0) {
            H1.setPower(0.3);
            H2.setPower(-0.3);
        } else if (gamepad1.left_trigger != 0) {
            H1.setPower(-0.3);
            H2.setPower(0.3);
        } else {
            H1.setPower(0);
            H2.setPower(0);
        }

        //Hanging
//        if (gamepad1.circle){
//            S0.setPosition(0.7);
//            S1.setPosition(0.7);
//        }
//        if (gamepad1.right_bumper){
//            H1.setPower(0.7);
//            H2.setPower(0.7);
//        }
//        else if (gamepad1.left_bumper){
//            H1.setPower(-0.7);
//            H2.setPower(-0.7);
//        }
//        else{
//            H1.setPower(0);
//            H2.setPower(0);
//        }
        //Drone

        //placing
        if(gamepad1.circle){

            S2.setPosition(1);
            S3.setPosition(0);
        }else if(gamepad1.square){
            S2.setPosition(0);
            S3.setPosition(1);
        }

    }
}
