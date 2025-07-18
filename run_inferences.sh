#!/bin/bash

run_inference() {
  local ROBOT=$1
  local EXPERIMENT_NAME=$2
  local TASK_TYPE=$3
  local TERRAIN_TYPE=$4
  local TERRAIN_DIFFICULTY=$5
  local MAX_SIM_TIME=$6
  shift 6

  local POLICY_PATH="models/${ROBOT}_${TASK_TYPE}/policy.pt"
  local SAVE_DIR="logs/inference/${ROBOT}_${TASK_TYPE}"
  local SAVE_FILENAME="${ROBOT}_${TASK_TYPE}_${TERRAIN_TYPE}_${EXPERIMENT_NAME}.pt"

  python scripts/play_inference.py \
      --robot "$ROBOT" \
      --task_type "$TASK_TYPE" \
      --policy_path "$POLICY_PATH" \
      --terrain "$TERRAIN_TYPE" \
      --terrain_difficulty "$TERRAIN_DIFFICULTY" \
      --max_sim_time "$MAX_SIM_TIME" \
      --save_data \
      --save_path "$SAVE_DIR" \
      --save_filename "$SAVE_FILENAME" \
      "$@"
}

# EXPERIMENTS
run_vel_tracking_experiments()
{
  local ROBOTS=("$@")
  for ROBOT in "${ROBOTS[@]}"; do
    # Predefined velocity commands
    run_inference "$ROBOT" predefined_cmds cpg_blind flat 1.0 30.0 --use_predefined_cmds --device cpu --headless

    # Higher velocity commands (with and without PI controller)
    run_inference "$ROBOT" higher_velocities cpg_blind flat 1.0 35.0 --use_higher_velocities --device cpu --headless
    run_inference "$ROBOT" higher_velocities_pi cpg_blind flat 1.0 35.0 --use_higher_velocities --use_pi_controller --device cpu --headless

    # Square trajectory (with and without PI controller)
    run_inference "$ROBOT" square_trajectory cpg_blind flat 1.0 25.0 --use_square_trajectory --device cpu --headless
    run_inference "$ROBOT" square_trajectory_pi cpg_blind flat 1.0 25.0 --use_square_trajectory --use_pi_controller --device cpu --headless

    # ZigZag trajectory (with and without PI controller)
    run_inference "$ROBOT" zigzag_trajectory cpg_blind flat 1.0 25.0 --use_zigzag_trajectory --device cpu --headless
    run_inference "$ROBOT" zigzag_trajectory_pi cpg_blind flat 1.0 25.0 --use_zigzag_trajectory --use_pi_controller --device cpu --headless

    # Without joint offsets
    run_inference "$ROBOT" cpg_only_flat cpg_blind flat 1.0 5.0 --only_cpg --use_predefined_cmds --device cpu --headless

  done
}

run_external_disturbances_experiments()
{
  local ROBOTS=($1)
  shift
  local TASKS=($1)
  shift

  for ROBOT in "${ROBOTS[@]}"; do
    for TASK in "${TASKS[@]}"; do
      for VEL_X in $(seq -0.5 -0.2 -2.5); do
        VEL_X_FMT=$(printf "%.1f" "$VEL_X")
        VEL_X_ABS=$(awk -v x="$VEL_X_FMT" 'BEGIN { print (x<0?-x:x) }')
        NAME="disturbance_x_${VEL_X_ABS//./}"

        run_inference "$ROBOT" "$NAME" "$TASK" random 1.0 5.0 \
          --use_predefined_cmds \
          --push_robot \
          --data_collection_interval 1.0 \
          --push_interval 3.0 \
          --define_pushes \
          --push_vel_x "$VEL_X_FMT" \
          --num_envs 1000 \
          --headless
      done

      for VEL_Y in $(seq 0.5 0.2 2.5); do
        VEL_Y_FMT=$(printf "%.1f" "$VEL_Y")
        VEL_Y_ABS=$(awk -v x="$VEL_Y_FMT" 'BEGIN { print (x<0?-x:x) }')
        NAME="disturbance_y_${VEL_Y_ABS//./}"

        run_inference "$ROBOT" "$NAME" "$TASK" random 1.0 5.0 \
          --use_predefined_cmds \
          --push_robot \
          --data_collection_interval 1.0 \
          --push_interval 3.0 \
          --define_pushes \
          --push_vel_y "$VEL_Y_FMT" \
          --num_envs 1000 \
          --headless
      done

      for VEL_YAW in $(seq 1.57 0.471 6.28); do
        VEL_YAW_FMT=$(printf "%.1f" "$VEL_YAW")
        VEL_YAW_ABS=$(awk -v x="$VEL_YAW_FMT" 'BEGIN { print (x<0?-x:x) }')
        NAME="disturbance_yaw_${VEL_YAW_ABS//./}"

        run_inference "$ROBOT" "$NAME" "$TASK" random 1.0 5.0 \
          --use_predefined_cmds \
          --push_robot \
          --data_collection_interval 1.0 \
          --push_interval 3.0 \
          --define_pushes \
          --push_vel_yaw "$VEL_YAW_FMT" \
          --num_envs 1000 \
          --headless
      done

      run_inference "$ROBOT" "disturbance_random" "$TASK" random 1.0 5.0 \
          --use_predefined_cmds \
          --push_robot \
          --data_collection_interval 1.0 \
          --push_interval 3.0 \
          --num_envs 1000 \
          --headless
    done
  done
}

run_terrain_experiments()
{
  local ROBOTS=($1)
  shift
  local TASKS=($1)
  shift

  local TERRAINS=("random" "waves" "boxes" "slope" "slope_down" "stairs" "stairs_down")

  for ROBOT in "${ROBOTS[@]}"; do
    for TASK in "${TASKS[@]}"; do
      for TERRAIN in "${TERRAINS[@]}"; do
        if [ "$TERRAIN" == "random" ]; then
          CMD=(run_inference "$ROBOT" forward10 "$TASK" "$TERRAIN" 1.0 30.0 \
            --use_forward_cmds --data_collection_interval 10.0 --save_interval 10.0 \
            --num_envs 1000 --headless)

          "${CMD[@]}"
        else
          for i in {0..10}; do
            local DIFFICULTY=$(echo "scale=1; $i / 10" | bc)
            local FORWARD_TASK="forward$i"

            CMD=(run_inference "$ROBOT" "$FORWARD_TASK" "$TASK" "$TERRAIN" "$DIFFICULTY" 30.0 \
              --use_forward_cmds --data_collection_interval 10.0 --save_interval 10.0 \
              --num_envs 1000 --headless)

            "${CMD[@]}"
          done
        fi
      done
    done
  done
}

run_vel_tracking_experiments "go2" "spot" "anymal_d"
run_external_disturbances_experiments "go2 spot anymal_d" "cpg_blind joints_blind"
run_terrain_experiments "go2 spot anymal_d" "cpg_blind cpg_vision joints_blind joints_vision"
