% trajectoy
cnt = 1;
while cnt<10
    trajectory = JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, [1 1 0.3 0 0 0]', time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);
    trajectory = JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, [1 1 1 0 0 0]', time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);

    trajectory = JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, [1 1 0.5 0 0 0]', time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);

    trajectory = JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, [0.8 1 1 0 0 0]', time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);

    trajectory = JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, [1.2 1 1 0 0 0]', time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);
    trajectory = JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, [1 1.1 1 0 0 0]', time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);


    trajectory = JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, [1 0.8 1 0 0 0]', time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);

    trajectory = JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, [1 1 1 0 0 0]', time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);
end