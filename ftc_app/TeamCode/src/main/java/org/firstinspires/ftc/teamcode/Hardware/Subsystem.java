package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {
    void init(HardwareMap hwMap, Telemetry tele);
    void start();
    void reset();
    void stop();
    State getState();
}