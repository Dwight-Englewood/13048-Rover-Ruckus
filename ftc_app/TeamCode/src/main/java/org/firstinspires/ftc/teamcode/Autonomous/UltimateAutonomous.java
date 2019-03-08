package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;
import org.firstinspires.ftc.teamcode.Hardware.bot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.sun.tools.javac.util.ForwardingDiagnosticFormatter;
import com.vuforia.CameraDevice;

import java.util.concurrent.ForkJoinPool;

@Disabled
@Autonomous(name="[New]AutonCraterUltimate", group="Autonomous")
public class UltimateAutonomous extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    bot robot = new bot();
    TensorFlow tensorFlow = new TensorFlow();

    TensorFlow.TFState BigThonk, actualState;

    int auto = 0;

    int center = 150;
    int left = 600;
    int right = 350;

    int centerBack = 1100;
    int leftBack = 800;
    int rightBack = 1750;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        tensorFlow.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.hook.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lift.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.claw.setPosition(0.0);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        tensorFlow.start();
        BigThonk = tensorFlow.getState();

        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (auto) {
            case 0:
                CameraDevice.getInstance().setFlashTorchMode(true);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.hook.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.claw.setPosition(0.0);
                auto++;
                break;

            case 1:
                robot.hook.setTargetPosition(7055);
                robot.hook.setPower(1);
                robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //  BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();

                if(!robot.hookLimit.getState() || robot.hook.getCurrentPosition() >= robot.hook.getTargetPosition()){
                    robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                    robot.hook.setPower(0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 2:
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                if(Math.abs(0- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(0);
                }
                else if(Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;

            case 3:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 4:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 140, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 5:
                BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();
                if(BigThonk != TensorFlow.TFState.NOTVISIBLE){auto++;}
                else {
                    BigThonk = TensorFlow.TFState.RIGHT;
                    auto++;
                }
                break;

            case 6:
                CameraDevice.getInstance().setFlashTorchMode(false);
                tensorFlow.stop();
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 7:
                robot.autonDriveUltimate(MovementEnum.RIGHTSTRAFE, 280, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 8:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 9:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(BigThonk == TensorFlow.TFState.CENTER){
                    auto++;
                }else if(BigThonk == TensorFlow.TFState.LEFT){
                    auto = 100;
                }else if(BigThonk == TensorFlow.TFState.RIGHT){
                    auto = 1000;
                }
                break;

                /*
           CASE FOR CENTER
           */
            case 10:
                if(Math.abs(-70- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-70);
                }
                else if(Math.abs(-70 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto = 13;
                }
                break;

                /*
           CASE FOR LEFT
           */
            case 100:
                if(Math.abs(-40- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-40);
                }
                else if(Math.abs(-40 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto = 13;
                }
                break;

                /*
           CASE FOR RIGHT
           */
            case 1000:
                if(Math.abs(-120- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-120);
                }
                else if(Math.abs(-120 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto = 13;
                }
                break;

            case 13:
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                tensorFlow.stop();
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 14:
                robot.autonDriveUltimate(MovementEnum.FORWARD, 500, 0.4);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 15:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 16:
                robot.dumpEntry.setPosition(0.0);
                robot.intake.setPower(1);
                robot.intake.setTargetPosition(2500);
                robot.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (Math.abs(robot.intake.getCurrentPosition()) >= Math.abs(robot.intake.getTargetPosition())){
                    robot.intake.setPower(0);
                    auto++;
                }
                break;

            case 17:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 18:
                robot.dumpEntry.setPosition(0.0);
                robot.intake.setPower(1);
                robot.intake.setTargetPosition(-200);
                robot.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (Math.abs(robot.intake.getCurrentPosition()) >= Math.abs(robot.intake.getTargetPosition())){
                    robot.intake.setPower(0);
                    auto++;
                }
                break;

            case 19:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 20:
                robot.dumpEntry.setPosition(0.75);
                robot.lift.setPower(1);
                robot.lift.setTargetPosition(400);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (Math.abs(robot.lift.getCurrentPosition()) >= Math.abs(robot.lift.getTargetPosition())){
                    robot.lift.setPower(0);
                    auto++;
                }
                break;

            case 21:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;


            case 22:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(BigThonk == TensorFlow.TFState.CENTER){
                    auto++;
                }else if(BigThonk == TensorFlow.TFState.LEFT){
                    auto = 100;
                }else if(BigThonk == TensorFlow.TFState.RIGHT){
                    auto = 1000;
                }
                break;

                /*
           CASE FOR CENTER
           */
            case 23:
                if(Math.abs(-60- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-60);
                }
                else if(Math.abs(-60 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;

                /*
           CASE FOR LEFT
           */
            case 200:
                if(Math.abs(-40- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-40);
                }
                else if(Math.abs(-40 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto = 24;
                }
                break;

                /*
           CASE FOR RIGHT
           */
            case 2000:
                if(Math.abs(-120- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-120);
                }
                else if(Math.abs(-120 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto = 24;
                }
                break;

            case 24:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 25:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 375, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 26:
                robot.dump.setPosition(0.75);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 27:
                robot.autonDriveUltimate(MovementEnum.FORWARD, 250, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 28:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(BigThonk == TensorFlow.TFState.CENTER){
                    auto++;
                }else if(BigThonk == TensorFlow.TFState.LEFT){
                    auto = 300;
                }else if(BigThonk == TensorFlow.TFState.RIGHT){
                    auto = 3000;
                }
                break;

                /*
           CASE FOR CENTER
           */
            case 29:
                if(Math.abs(-120- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-80);
                }
                else if(Math.abs(-80 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;

                /*
           CASE FOR LEFT
           */
            case 300:
                if(Math.abs(-140- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-120);
                }
                else if(Math.abs(-120 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto = 30;
                }
                break;

                /*
           CASE FOR RIGHT
           */
            case 3000:
                if(Math.abs(-80- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-40);
                }
                else if(Math.abs(-40 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto = 30;
                }
                break;

            case 30:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 31:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(BigThonk == TensorFlow.TFState.CENTER){
                    auto++;
                }else if(BigThonk == TensorFlow.TFState.LEFT){
                    auto = 400;
                }else if(BigThonk == TensorFlow.TFState.RIGHT){
                    auto = 4000;
                }
                break;

                /*
           CASE FOR CENTER
           */
            case 32:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, centerBack, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

                /*
           CASE FOR LEFT
           */
            case 400:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, leftBack, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto = 33;
                }
                break;

                /*
           CASE FOR RIGHT
           */
            case 4000:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, rightBack, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto = 33;
                }
                break;

            case 33:
                if(Math.abs(-135 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-135);
                }
                else if(Math.abs(-135 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;

            case 34:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 35:
                robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, 350, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 36:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 37:
                robot.autonDriveUltimate(MovementEnum.RIGHTSTRAFE, 100, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 38:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 39:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 1000, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 40:
                robot.claw.setPosition(0.7);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(robot.claw.getPosition() >= 0.7) {
                    robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    auto++;
                }
                break;

            case 41:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 42:
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, 100, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 43:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 44:
                robot.autonDriveUltimate(MovementEnum.FORWARD, 2100, 0.3);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                }
                break;

        }
        telemetry.addData("Case Number:", auto);

        telemetry.addData("Lift Power", robot.lift.getPower());
        telemetry.addData("Lift curPos", robot.lift.getCurrentPosition());
        telemetry.addData("Lift tarPos", robot.lift.getTargetPosition());

        telemetry.update();
    }
}