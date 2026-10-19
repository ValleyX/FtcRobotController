package org.firstinspires.ftc.team2844.commands.turret;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.subsystems.TurretSubsystem;
import org.firstinspires.ftc.team2844.subsystems.VisionSubsystem;

import java.util.Collections;
import java.util.Set;

public class DefaultTurretCmd implements Command {
    TurretSubsystem turret;
    VisionSubsystem vision;
    boolean track;
    boolean manual;
    boolean relative;
    double degrees;

    public DefaultTurretCmd(TurretSubsystem turret, VisionSubsystem vision){
        this.turret = turret;
        this.vision = vision;
        track = false;
        manual = false;
        relative = false;
        degrees = 0.0;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(turret);
    }

    @Override
    public void initialize() {
        turret.setRelativeDegrees(0.0);
    }

    @Override
    public void execute() {
        if(manual){
            if(relative) {
                turret.setRelativeDegrees(degrees);
            } else {
                turret.setFieldCentricDegrees(degrees);
            }
        } else if(track){
            if(vision.valid()){
                turret.aimToTx();
            } else {
                turret.aimToTarget();
            }
        } else {
            turret.setRelativeDegrees(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setRelativeDegrees(0.0);
    }

    /* ---------- GETTERS AND SETTERS ---------- */
    //relative boolean
    public void setRelative(boolean relative){
        this.relative = relative;
    }
    public boolean getRelative(){
        return relative;
    }

    //Degrees
    public void setDegrees(double degrees){
        this.degrees = degrees;
    }
    public double getDegrees(){
        return degrees;
    }

    //Boolean Manual
    public void setManual(boolean manual){
        this.manual = manual;
    }
    public boolean getManual(){
        return manual;
    }

    //Boolean Track
    public void setTrack(boolean track){
        this.track = track;
    }
    public boolean getTrack(){
        return track;
    }
}
