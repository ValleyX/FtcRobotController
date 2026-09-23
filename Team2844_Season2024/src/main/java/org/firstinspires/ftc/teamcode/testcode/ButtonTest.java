package org.firstinspires.ftc.team12841.testcode;
import com.arcrobotics.ftclib.command.CommandBase;

//TESTCODE
public class ButtonTest extends CommandBase {
    private final FieldCentricTestDriveSubsystem m_driveSubsytem;
    public ButtonTest(FieldCentricTestDriveSubsystem driveSubsystem){
        m_driveSubsytem = driveSubsystem;
        addRequirements(m_driveSubsytem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_driveSubsytem.allPower(.5);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
