package org.firstinspires.ftc.teamcode.PIDStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.BoBot;
//<<<<<<< HEAD:ftc_app/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Hardware/PIDController.java
import java.math.RoundingMode;
import java.text.DecimalFormat;
//=======
import org.firstinspires.ftc.teamcode.PIDStuff.PID;
//>>>>>>> dc654ac7e5749e1816169ca58db0066117909110:ftc_app/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PIDStuff/PIDController.java

//@Disabled
public class PIDController {

    public double pGain;
    public double iGain;
    public double dGain;

    private double goal;
    public double error;

    private double lX;
    private double dError;
    private double iError;

    public PIDController(double pGain, double iGain, double dGain) {
        this.pGain = pGain;
        this.iGain = iGain;
        this.dGain = dGain;
    }

    public PIDController(double pGain, double iGain, double dGain, double goal) {
        this.pGain = pGain;
        this.iGain = iGain;
        this.dGain = dGain;
        this.goal = goal;
    }

    public static void main(String[] args) {
        int merp = 0;
        DecimalFormat df = new DecimalFormat("#.####");
        df.setRoundingMode(RoundingMode.CEILING);
        PIDController dab = new PIDController(.3, 0, 0);
        //PIDController dab = new PIDController(.6, 0, 2.5);
        //PIDController dab = new PIDController(.4, .001, 6.5);

        double currentPosition = 10;
        double currentVelocity = 0;
        double currentAcceleration = 0;
        final double mass = 100;
        final double gravity = 9.8;
        dab.setGoal(0);


    }

    public double correction() {
        return ((error * pGain) + (iError * iGain) - (dError * dGain));
    }

    public void updateError(double currentPosition) {
        this.error = goal - currentPosition;
        this.iError = this.iError + this.error;
        this.dError = currentPosition - this.lX;
        this.lX = currentPosition;
    }

    public void setGoal(double goal) {
        this.reset();
        this.goal = goal;
    }

    protected void setGoalNoReset(double goal) {
        this.goal = goal;
    }

    public boolean goalReached(double resolution) {
        return Math.abs(this.error) < resolution;
    }

    public void reset() {
        this.goal = 0;
        this.error = 0;
        this.lX = 0;
        this.dError = 0;
        this.iError = 0;
    }

}