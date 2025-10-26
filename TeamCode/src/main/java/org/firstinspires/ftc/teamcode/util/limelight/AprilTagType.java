package org.firstinspires.ftc.teamcode.util.limelight;

public enum AprilTagType {
    BLUE_GOAL,
    RED_GOAL,
    MOTIF_GPP,
    MOTIF_PGP,
    MOTIF_PPG;

    public static AprilTagType getAprilTagType(int id) {
        if(id==20) return BLUE_GOAL;
        if(id==21) return MOTIF_GPP;
        if(id==22) return MOTIF_PGP;
        if(id==23) return MOTIF_PPG;
        return RED_GOAL;
    }
    public boolean isMotif() {
        return this == MOTIF_GPP || this == MOTIF_PGP || this == MOTIF_PPG;
    }
    public int getTagIDNumber() {
        switch(this) {
            case BLUE_GOAL:
                return 20;
            case MOTIF_GPP:
                return 21;
            case MOTIF_PGP:
                return 22;
            case MOTIF_PPG:
                return 23;
            case RED_GOAL:
            default:
                return 24;
        }
    }
}
