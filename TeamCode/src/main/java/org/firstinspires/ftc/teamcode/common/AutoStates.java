package org.firstinspires.ftc.teamcode.common;

public class AutoStates {
    private int cycleNumber;
    private int artifactCnt;
    private boolean isRed;
    public int getCycleNumber() {
        return cycleNumber;
    }

    public void setCycleNumber(int cycleNumber) {
        this.cycleNumber = cycleNumber;
    }
    public int getArtifactCnt() { return artifactCnt; }
    public void setArtifactCnt(int artifactCnt) { this.artifactCnt = artifactCnt; }

    public boolean isRed() {
        return isRed;
    }

    public void setAllianceColor(boolean isRed) {
        this.isRed = isRed;
    }
}
