#!/bin/bash
success_count=0
LOG_FILE="experiment_results.log"
echo "Starting 5 iterations of object grasp experiment" > $LOG_FILE

for i in {1..5}; do
    echo "======================================" | tee -a $LOG_FILE
    echo "Running iteration $i..." | tee -a $LOG_FILE
    
    # Run the test, tee output so we can parse it and also save it
    bash scripts/run_test.sh 2>&1 | tee /tmp/iter_${i}.log
    
    if grep -q "Success: True" /tmp/iter_${i}.log; then
        echo "Iteration $i: SUCCESS" | tee -a $LOG_FILE
        success_count=$((success_count + 1))
    else
        echo "Iteration $i: FAILURE" | tee -a $LOG_FILE
    fi
    
    # Save the videos for this iteration
    if [ -f "red_cube_run.mp4" ]; then
        mv red_cube_run.mp4 iter_${i}_red_cube_run.mp4
    fi
    if [ -f "red_cube_run_overhead.mp4" ]; then
        mv red_cube_run_overhead.mp4 iter_${i}_red_cube_run_overhead.mp4
    fi
    
    # Sleep a bit between runs to let ports clear
    sleep 10
done

echo "======================================" | tee -a $LOG_FILE
echo "Total Successes: $success_count / 5" | tee -a $LOG_FILE
