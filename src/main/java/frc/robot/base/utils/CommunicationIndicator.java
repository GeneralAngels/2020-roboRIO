package frc.robot.base.utils;

import com.ga2230.shleam.advanced.frc.FRCModule;
import com.ga2230.shleam.base.structure.Function;
import com.ga2230.shleam.base.structure.Result;

public class CommunicationIndicator extends FRCModule {
    public CommunicationIndicator() {
        super("indicator");

        register("state", new Function() {
            @Override
            public Result execute(String parameter) throws Exception {
                return Result.finished(parameter);
            }
        });
    }
}
