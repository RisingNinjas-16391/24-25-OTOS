package org.firstinspires.ftc.teamcode.subsytems.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DrivetrainSubsystem extends SubsystemBase {
    private final SparkFunOTOSDrive m_drive;
    private final PIDFController kHeadingPID;
    private double desiredHeading = 0;
    public static double omegaSpeed = 0.5;
    private final boolean m_fieldCentric;
    private double m_headingOffset = 0;

    public DrivetrainSubsystem (@NonNull HardwareMap, HardwareMap, Boolean fieldCentric){
        m_drive = new SparkFunOTOSDrive(HardwareMap,0,0,Math.toRadians(0));/*
        kHeadingPID = new PIDFController(DriveConstraints);
        m_drive.setMode;*/
        m_fieldCentric = fieldCentric;
    }
    @Override
    public void periodic(){
        m_drive.updatePoseEstimate();
    }

    public void updateTelemtry(Telemetry telemetry){
        telemetry.addLine("Drivetrain");
        telemetry.addData("Pose",m_drive.pose);
    }
    public void setDrivePower(@NonNull Pose2d drivePower){
        Vector2d input = new Vector2d(drivePower.getY(), drivePower.getX());
        m_drive.setDrivePowers(new Pose2d(
                input.getX(),
                input.getY(),
                drivePower.getHeading()))
        ));
    }

    public void resetHeading(){
        m_headingOffset=getHead
    }

}