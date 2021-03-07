package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class WobbleGoal {
    // Set up Hardware Devices
    /* Motors */
    DcMotor linSlide;
    double slideLowerPower = 0.25; //1.0
    double slideLiftPower = 0.80;
    /*Servos*/
    Servo claw;
    /*** CLAW Positions ***/
    /**
     *  double lift = 0;
     *     double grab = 0.25;
     *     double stow = 0.5;
     */
    double lift = 0;
    double grab = 0.25;
    double stow = 0.5;

    //Constructors
    public WobbleGoal(DcMotor linearSlide, Servo servo){
        linSlide = linearSlide;
        claw = servo;
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //Methods

    /***SERVO METHODS***/
    public void setClaw(double pos) {
        this.claw.setPosition(pos);
    }
    public void storeClaw(){
        this.setClaw(stow);
    }
    public void activateClaw() {
        this.setClaw(grab);
    }
    public void setliftClawPosition(){
        this.setClaw(lift);
    }
    public void zeroPosition(){
        this.setClaw(0);
    }

    /*** LINEAR SLIDE METHODS ***/
    public void raiseWobbleGoal(){
        linSlide.setPower(slideLiftPower);
    }
    public void lowerWobbleGoal(){
        linSlide.setPower(-1*slideLowerPower);
    }
    public void stopGoal(){ linSlide.setPower(0);}
    public void deliverWobbleGoal(){
        //autonomous??
        //lower linear slide timed, encodered, ....??
        //move claw to its stored position
    }
}