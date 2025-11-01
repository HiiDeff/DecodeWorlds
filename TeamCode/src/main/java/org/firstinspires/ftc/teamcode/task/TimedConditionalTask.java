package org.firstinspires.ftc.teamcode.task;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Condition;

/**
 * A task tells if the operation should continue or terminate. This should only be used in a
 * SeriesTask.
 */
public class TimedConditionalTask extends TimedTask {
    private final Condition condition;
    public TimedConditionalTask(Condition condition, int finishTimeMillis) {
        this.condition = condition;
        setFinishTimeMillis(finishTimeMillis);
    }
    @Override
    public boolean performInternal() {
        // For other tasks, TRUE means task is done, FALSE means task not done. For
        // ConditionalTask, the task is performed only once. The meaning of the return value is
        // different from other tasks. TRUE means the next task in a SeriesTask can continue.
        // Otherwise, the SeriesTask should be terminated.
        return condition.shouldContinue();
    }

    @Override
    public void cancel() {
        Log.i("edbug", "no artifact detected!");
        super.cancel();
    }
}
