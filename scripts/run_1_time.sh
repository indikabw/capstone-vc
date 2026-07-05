#!/bin/bash
success_count=0
LOG_FILE="experiment_results_new_spawn.log"
echo "Starting 1 iteration of new spawn experiment" > $LOG_FILE

# Ensure the modified launch files are built
source /opt/ros/lyrical/setup.bash
bash scripts/colcon_build.sh --packages-select custom_bot_gazebo custom_bot_reasoning

echo "======================================" | tee -a $LOG_FILE
echo "Running iteration 1..." | tee -a $LOG_FILE

# Run the test
bash scripts/run_test.sh 2>&1 | tee /tmp/iter_new_spawn_1.log

if grep -q "Success: True" /tmp/iter_new_spawn_1.log; then
    echo "Iteration 1: SUCCESS" | tee -a $LOG_FILE
    success_count=$((success_count + 1))
else
    echo "Iteration 1: FAILURE" | tee -a $LOG_FILE
fi

# Save videos
if [ -f "red_cube_run.mp4" ]; then
    mv red_cube_run.mp4 iter_new_spawn_1_red_cube_run.mp4
fi
if [ -f "red_cube_run_overhead.mp4" ]; then
    mv red_cube_run_overhead.mp4 iter_new_spawn_1_red_cube_run_overhead.mp4
fi

echo "======================================" | tee -a $LOG_FILE
echo "Total Successes: $success_count / 1" | tee -a $LOG_FILE
