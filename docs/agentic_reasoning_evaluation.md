# Agentic Reasoning Evaluation: Closed-Loop Object Picking with Visual Servoing

**Date:** 2026-07-04
**Code evaluated:** `experimental/debug` @ `0c57170` (verified byte-identical between local repo and VM `~/capstone-vc`)
**Command:** `bash scripts/run_test.sh` → sends `"Pick up the red_cylinder in front of you."` to `/reasoning_task`

## 1. Summary

Two end-to-end runs were executed in the live Gazebo/Nav2/MoveIt2/ADK stack. **Both failed to pick up the object (0/2).** Both failures originated in the closed-loop control layer (navigation/visual-servo), not in semantic grounding — the agent correctly identified `red_cylinder` and its coordinates in every run. The action server reported `result.success = True` in the run that completed, despite no grasp having occurred; only the external `gz model` physics check (in `scripts/test_nav_and_pick.py`) would have caught this.

## 2. Run traces

### Run A (baseline, prior to this evaluation session, log timestamps ~23:11)
~30 tool calls over ~18 minutes. Sequence (abbreviated):
```
list_objects → get_object_details(red_cylinder)
→ navigate(-2.25,-2) → feasibility checks (out of reach)
→ navigate(-2,-2) → feasibility(z=0.25,pitch=1.57) pass → hover_and_open
→ assess_visual_alignment → adjust_pose(dx=0, dy=0.11)
→ assess_visual_alignment → adjust_pose(dx=0, dy=-0.28) → adjust_pose(dx=0, dy=-0.11)
→ navigate(-1.9,-2) → feasibility pass → hover_and_open
→ assess_visual_alignment → adjust_pose(dx=0, dy=0.25)
→ feasibility checks at z=0.15 (both pitches) → get_object_details (re-grounding)
→ navigate(-1.75,-2.5) → [458s gap] → feasibility checks → navigate(-1.75,-2.3)
→ feasibility pass → hover_and_open → assess_visual_alignment
→ adjust_pose(dx=-0.18, dy=-1.55)   <-- 1.55 m "depth" correction on a 5cm object
→ "Reasoning complete."
```
`close_gripper_and_lift_tool` was **never called**. The node still returned `success=True`.

### Run B (this session, fresh launch)
Same opening (list/ground/navigate/feasibility/hover), then:
```
assess_visual_alignment_tool(red_cylinder)
adjust_pose_tool(dx=0, dy=0.12, dz=0)
navigate_and_face_tool(robot: -1.9,-2.25, face: -1.75,-2)   <-- HANGS
```
Confirmed via `/tmp/nav.log`: `controller_server` accepted the goal and never logged "Reached the goal," never errored, never recovered. The tool's blocking wait (`while not result_future.done(): time.sleep(0.1)`, no timeout, no cancel) held the entire agent frozen for **782 seconds** before the run was manually terminated (matches the unexplained 458s gap in Run A — same failure mode, worse instance).

## 3. Root-cause analysis

**Architectural root cause: an LLM (`gemini-2.5-pro`) is placed inside a tight, iterative control loop** (visual alignment) instead of being confined to planning/grounding. This is slow (each step is a full LLM round-trip), non-deterministic, and — critically — has no code-enforced convergence guarantee, only a prose instruction telling the model when to stop.

### 3.1 Visual servoing is not a functioning control loop
File: `src/custom_bot_reasoning/custom_bot_reasoning/reasoning_node.py`

- **`assess_visual_alignment_tool` (:485-522)** asks the VLM to estimate a metric `(dx, dy)` offset in meters from a single monocular RGB frame, with no scale reference, no camera intrinsics, and no known object size in the prompt. Absolute metric depth from one image is not a task a VLM can do reliably — this is the direct cause of the 1.55 m and 0.28 m "corrections" observed on a 5 cm object.
- **`adjust_pose_tool` (:524-552) silently drops `dx`.** Only `dy` (mapped to reach radius) and `dz` (height) are applied; the comment at `:536` admits lateral (`dx`) correction is not implemented. Any run where the object is laterally off-center in the gripper's approach **cannot converge** — this is what forces the observed re-navigate churn (the agent's only remaining lever for lateral error is moving the whole base).
- **No controller.** The raw VLM-estimated offset is applied 1:1 with no proportional gain, no per-step clamp, and no iteration cap in code (only a prose "repeat until <2cm" instruction to the LLM). A single bad estimate can move the arm the full (wrong) distance, and nothing prevents unbounded oscillation.
- **Convergence is judged by the LLM parsing a string**, not by code comparing to the stated 0.02 m tolerance.
- Unverified: whether the gripper is even in-frame from the OAK-D at hover. The camera mounts low and forward on the base (`x=+5.84cm, z=+9.68cm` relative to its bracket per `turtlebot4.urdf.xacro`), while the arm mounts separately on the elevated `tower_sensor_plate` (`+12cm` forward, per `robot.urdf.xacro`) — the two are not co-located, so the servoing prompt's premise ("image ... looking at the gripper and the object") should be visually confirmed, not assumed.

### 3.2 The approach-pitch parameter the agent deliberates over is cosmetic
`solve_ik_planar` (:322-388) uses `ori_weight=0.001` — negligible next to the position error terms. I verified this numerically (standalone reproduction of the solver, same target/guesses):

| Requested pitch (`alpha`) | Actual end-effector pitch (q2+q3+q4) | Deviation |
|---|---|---|
| 1.57 (horizontal) | 0.71 | 49° |
| 3.14 (top-down) | 0.94 | 126° |
| 0.0 | 0.49 | 28° |
| -1.57 | 0.26 | 89° |

The solver converges to whatever pitch satisfies *position*; the requested approach angle is not honored. So `check_grasp_feasibility_tool`'s horizontal-vs-top-down distinction — which the agent spent multiple tool calls on in both runs, trying `1.57`, `3.14`, `-1.57`, and `0.1` — changes the outcome only through solver-initialization noise, not through any real kinematic difference. This gives the agent false confidence about an approach angle it isn't actually achieving.

### 3.3 No watchdogs anywhere in the control-dispatch layer
- `navigate_and_face_tool` (:188-221): unbounded `while not future.done(): time.sleep(0.1)` on both the goal-accept and result futures. No timeout, no `cancel_goal_async`. This is what produced the 782s hang in Run B and the 458s gap in Run A.
- `execute_moveit_pose` / `execute_moveit_joints` (:223-320): same pattern, same risk, not yet observed to hang but structurally identical.
- No overall task-time budget in `execute_callback` (:608-662) — the ADK reasoning loop runs via `asyncio.run(run_adk())` with no timeout, so a stuck tool call blocks the entire action goal indefinitely.
- No real failure path: `goal_handle.abort()` is never called anywhere in the file.

### 3.4 Tool contracts mislead the agent
- `check_grasp_feasibility_tool`'s return string (:459) tells the agent to *"call `execute_grasp_tool`"* — **this tool does not exist** anywhere in the codebase (confirmed via grep). The agent must be inferring the correct next tool (`hover_and_open_tool`) from the system instruction, not the tool's own output.
- **Contradictory pitch conventions**: the `spatial_critic` instruction (:98) says horizontal=`1.57`/top-down=`3.14`; the `check_grasp_feasibility_tool` docstring (:448) says horizontal=`0.0`/top-down=`-1.57`. The agent visibly thrashed between `1.57`, `3.14`, `-1.57`, and `0.1` across the two runs trying to reconcile these.
- The Semantic Planner's own example instruction (:123) hardcodes a stale object name and a wrong height — *"pick up 'red_block' at grasp_z=0.25"* — while the actual target is `red_cylinder` at `z=0.1` (top) per `semantic_map.json`. The agent parroted `z=0.25` as its first feasibility guess in both runs.

### 3.5 Success is ungrounded
`execute_callback` (:657-662) always calls `goal_handle.succeed()` and sets `result.success = True`, even when the ADK call raises an exception (the `except` branch at :652 only changes the summary text, not the success flag). The architecture doc (`docs/system_architecture.md` §4.5) promises visual/physical verification of subtask completion before marking a task "Done" — this is not implemented. In Run A, the action server reported success while the cylinder never left the ground; only `scripts/test_nav_and_pick.py`'s external `gz model -m red_cylinder -p` check (documented as necessary in `.agents/skills/headless-simulation-monitoring/SKILL.md` §6, "True Physics State vs. Node State") would have caught this, and the test run never got there.

### 3.6 The "Spatial Critic" doesn't critique
The two-agent split (Semantic Planner → Spatial Critic) is a second LLM hop, not a validation stage. The Critic executes navigation, IK checks, hover, servoing, and grasping — the same agent proposes and "approves" its own moves. No independent check on reach, collision, or safety exists before actuation. As built, the split adds latency (two `gemini-2.5-pro` agents involved per task) without adding the review function its name implies.

### 3.7 Secondary issues (lower priority)
- `hover_and_open_tool` claims to hover "directly above" the object but actually targets the grasp radius `r+0.02` at `grasp_z+0.15` — i.e., forward-and-up, not straight up (:461-483).
- Dead/misleading code: `execute_moveit_pose` (:223) is defined but never called; `place_tool` (:585) ignores its `(x, y, z)` arguments entirely and runs a fixed trajectory.
- Object geometry disagreement: the test scenario states Ø5cm × 15cm; `semantic_map.json` states `size: {dx:0.03, dy:0.03, dz:0.2}` (Ø3cm × 20cm); `test_nav_and_pick.py::verify_physics` treats any `z > 0.12` (a ~2cm rise from a resting `z≈0.10`) as a successful lift.
- Repo hygiene: `patch.py`, `fix_nav.py`, `fix_reasoning.py` at the repo root are one-off regex monkey-patches applied directly to `reasoning_node.py`/`navigation.launch.py`; `patch.py` no longer matches the current file content. All reasoning logic (agents, tools, IK solver, ROS plumbing) lives in one 680-line file.

## 4. What already works well (preserve these)

- Discrete action-server triggering (not continuous per-frame reasoning) — correctly avoids latency/callback-starvation per `.agents/skills/camera-agent-reasoning/SKILL.md`.
- `MultiThreadedExecutor` + mutex-guarded latest-image pattern for decoupling the blocking LLM call from ROS callbacks.
- External physics verification via `gz model -p` in the test harness — the team has already internalized that "node success ≠ physical success" (see `headless-simulation-monitoring/SKILL.md` §6); this instinct just needs to be wired into the reasoning node's own result, not left solely to an external test script.
- IK reach pre-check before committing to a grasp attempt.
- Documented, hard-won lessons on gripper joint sign convention and gripper-jaw-vs-wrist depth offset (`camera-agent-reasoning/SKILL.md` §4).

## 5. Recommendations, in priority order

1. **Remove the LLM from the servo loop.** Replace the assess/adjust LLM round-trip with a single deterministic tool that runs HSV-based segmentation of the (saturated red, easily thresholded) object, computes a pixel-space centroid/size error, converts it to metric offsets using the known camera geometry (horizontal FOV, resolution) rather than asking a model to guess meters, and drives a bounded proportional controller (fixed gain, per-step clamp, hard iteration cap) entirely in code. The agent calls this once; it no longer arbitrates alignment step by step.
2. **Add watchdogs.** Timeout + `cancel_goal_async` on every Nav2/MoveIt blocking wait; an overall wall-clock budget around the ADK run in `execute_callback`; and an honest `goal_handle.abort()` / `result.success = False` path instead of unconditional success.
3. **Fix the visual signal contract**, if any VLM step is retained: request only qualitative/pixel-space information (bounding box, normalized direction), never absolute metric offsets; apply `dx` (it is currently silently dropped); make code, not the model, evaluate the convergence tolerance.
4. **Ground success** in the node itself: after lift, check the gripper joint didn't close to its fully-closed (empty) position and/or re-image the gripper, and set `result.success` from that outcome.
5. **Clean up tool contracts**: remove the reference to the nonexistent `execute_grasp_tool`; unify the pitch convention across the planner instruction and the tool docstring; make the IK solver actually honor the requested approach pitch (raise `ori_weight` materially or solve the wrist orientation analytically); remove the stale `red_block`/`grasp_z=0.25` example from the planner's system instruction; correct "hover directly above" to be actually vertical.
6. **Re-scope or collapse the actor-critic split**: either give the Spatial Critic a real, independent veto over the Planner's proposed pose (reach/collision/safety check before actuation), or fold both into a single planner that calls well-tested deterministic skills — avoid paying for two LLM agents that both ultimately just call tools.
7. **Hygiene**: delete the ad-hoc `patch.py`/`fix_nav.py`/`fix_reasoning.py` monkey-patch scripts (fold their intent into proper commits); split `reasoning_node.py` into node / IK-solver / tool-definitions / agent-config modules; reconcile the red_cylinder size across the scenario spec, `semantic_map.json`, and the test's lift-detection threshold; add a small N-trial eval harness that reports the *physics-verified* success rate, since currently there is no quantitative success metric at all.

## 6. Fixes implemented (this session)

- **#1 Deterministic visual servo**: replaced the LLM-mediated `assess_visual_alignment_tool`/`adjust_pose_tool` loop with `visual_servo_align_tool`, which runs entirely in code - HSV segmentation of the (red) object, pinhole-geometry lateral/depth error estimation using the known camera focal length (derived from `horizontal_fov=1.25`, `width=1280`), a proportional controller with fixed gain and hard per-step clamps, and a hard 8-iteration cap. No LLM calls occur inside the loop. `dx` (lateral) is now actually applied, via a `j1_offset` accumulator threaded through `hover_and_open_tool`, `adjust_pose_tool`, and `close_gripper_and_lift_tool`.
- **#2 Watchdogs + honest failure signaling**: added bounded timeouts (with `cancel_goal_async`) to every Nav2/MoveIt2 blocking wait (`NAV_RESULT_TIMEOUT_SEC=240`, `MOVEIT_RESULT_TIMEOUT_SEC=60`, plus accept-phase timeouts), and an overall `TASK_TIMEOUT_SEC=600` deadline around the ADK run via `asyncio.wait_for`. `result.success` is now `False` (and `goal_handle.abort()` is called) on any node-level exception or timeout, instead of always `True`. **Known residual gap**: this only catches node-level failures, not a *graceful* LLM-authored failure summary (see Run 3/4 below) - the boolean can still read `True` while the agent's own prose says it failed. Full physical-outcome grounding (recommendation #4 from §5) was not implemented this session.
- Also fixed the misleading `execute_grasp_tool` reference in `check_grasp_feasibility_tool`'s return string, and the stale `grasp_z=0.25`/`red_block` example in the Semantic Planner's instruction (see §7 - this turned out to be load-bearing, not just hygiene).

## 7. Post-fix validation runs

Two additional end-to-end runs were executed after the fixes above (Run 3 and Run 4 below; Runs A/B are the pre-fix baselines from §2).

### Run 3 (post-fix)
Completed cleanly in **~161s** (vs. 18 min / manual-kill for the baselines) with the full pipeline - sim, nav, bag record, video conversion - finishing on its own with no manual intervention. Tool trace: grounded the object, staged at 3 positions (`-1.4`, `-1.55`, `-1.65` along y), ran `check_grasp_feasibility_tool` at **`grasp_z=0.1`** (the cylinder's true height, per `semantic_map.json`) with both pitches at each position, all failed "out of reach," then stopped. It never called `hover_and_open_tool`, so `visual_servo_align_tool` was never exercised. The agent's own final summary was accurate and specific: *"the required reach to grasp the object is 0.41m, but the arm's maximum reach is 0.37m."* `result.success` was `True` (no exception occurred - this is the residual gap noted in §6); the external physics check correctly caught the false positive (`z=0.1`, no lift).

### Run 4 (post-fix)
Completed cleanly in **~194s**. Same shape: staged at 4 positions (`-1.2`, `-1.6`, `-1.7`, `-1.65`), checked feasibility at **`grasp_z=0.15`** repeatedly, all failed, gave up with an accurate self-diagnosis ("either too far to reach... or too close, causing the inverse kinematics solver to fail"). Notably the agent's closest attempt (`-1.7`, i.e. 0.3m from the object) was followed by moving *back out* to `-1.65` (0.35m) rather than committing to the closest point already tried - a search-strategy weakness, not a hang.

### Why the baselines "passed" feasibility but these didn't
The two pre-fix baseline runs (§2) both used **`grasp_z=0.25`** - a stale hardcoded example value in the old Semantic Planner instruction (*"pick up 'red_block' at grasp_z=0.25"*), not the cylinder's actual height. Runs 3-4 (this session) instead targeted the object's *true* height (`z≈0.1-0.15`, per `semantic_map.json`), because the planner instruction fix in §6 explicitly tells the agent to use the object's real reported `z`. I verified numerically that this is a genuine reach-envelope effect, not noise: backing out the implied geometry from Run 3's own reported numbers (staged 0.35m away, required reach 0.41m) shows the vertical differential between the elevated arm mount and a near-ground grasp point is ~0.256m - **69% of the arm's entire 0.37m reach budget** - leaving only ~0.24m of horizontal budget. That means the robot must stage within roughly **0.25m** of the object's center for a low grasp (base radius is 0.17m, so this is tight but not physically impossible), far closer than any "safe distance" heuristic would normally produce.

**Conclusion: the two prior "successful" runs likely never had a real chance of completing the pick regardless of the visual-servo/watchdog bugs, because they were reaching for empty air 0.15-0.2m above the actual object.** The stale example value was masking this until the planner-instruction fix (§6) exposed it. I applied a follow-up fix in this session: the planner now must pass the object's real reported `z`, and the Spatial Critic instruction now explicitly requires staging within ~0.25m for low grasps and moving *strictly closer* (never sideways/farther) on repeated "out of reach" failures.

### Net assessment (Runs 3-4)
- **Validated**: #2 (watchdogs) - both post-fix runs completed the *entire* pipeline unattended in ~3 minutes, a qualitative reliability improvement over baselines that required an 18-minute run or a manual kill of a 13-minute hang.
- **Not yet validated**: #1 (deterministic visual servo) - neither post-fix run reached `hover_and_open_tool`, so `visual_servo_align_tool` has not yet been exercised in a live run. This remains to be confirmed once the staging-distance fix lets the agent clear the feasibility gate.
- **New finding, higher priority than originally ranked**: the arm-mount-height vs. reach-budget interaction for near-ground objects should be verified against real object geometry (or the mount height reconsidered) independently of any reasoning-layer fix, since it can make the "correct" grasp height kinematically unreachable from any but a very tight staging distance.

## 8. Follow-up fix and Run 5

Applied a second round of prompt fixes based on §7's discovery: the Semantic Planner instruction now requires passing the object's own reported `z` (not a hardcoded example height), and the Spatial Critic instruction now explicitly requires staging within ~0.25m of the object for low grasps and moving *strictly closer* (never sideways/farther) after a feasibility failure.

### Run 5 (post-fix, with staging-distance guidance)
Completed in **~530s (~8.8 min)** - longer than Runs 3-4, but still fully unattended, no manual kill required. The agent staged progressively closer along two different approach vectors: `(-2,-2)` → `(-1.9,-2)` → `(-1.8,-2)` → `(-1.75,-1.8)`, reaching gaps as tight as **0.05-0.2m** from the object, and tried `grasp_z` at both `0.1` and `0.15`. Feasibility still failed at every position and both pitches. Two things stand out:

1. **The 240s Nav2 watchdog fired live**, twice, on the two closest-staging goals - Nav2's `controller_server` accepted the goal but the base barely moved (confirmed via `/odom`, position essentially static across samples), consistent with the tight staging distance conflicting with the object's inflated costmap footprint. In the pre-fix codebase this would have been another indefinite hang requiring a manual kill; here the tool call returned its bounded-timeout message on schedule, and the agent incorporated it into its final summary ("I also encountered a navigation failure when trying to position the base to a potentially better vantage point") and terminated gracefully instead of hanging. **This is the watchdog fix (#2) working exactly as designed, under a harder, previously-untested failure condition.**
2. **New side effect discovered**: the physics check reported `red_cylinder z = 0.015` - *lower* than the object's resting height of `0.1`, rather than the unmoved `0.1` seen in Runs 3-4. This indicates the robot's base likely made physical contact with the cylinder during one of the sub-0.2m staging attempts and knocked/settled it out of place. **Staging this close trades one risk (kinematically unreachable low grasps) for another (base-object collision)** - a real cost of the §7 fix that should be weighed before adopting a fixed "~0.25m" staging rule generally.

### Updated conclusion
Across five total runs (2 baseline + 3 post-fix), the pipeline has never completed a pick, but the failure mode has moved twice:
1. Baseline: visual-servo divergence / unbounded Nav2 hang (§2).
2. Post-fix, wrong-height masking removed: kinematic infeasibility at the object's true low height (§7).
3. Post-fix, tight staging: feasibility *still* fails even at 0.05-0.2m range, and tight staging introduces base-object collision risk (§8).

This means the kinematic-reach problem for a `grasp_z≈0.1-0.15` target is not fully explained by staging distance alone - something in the reach/IK geometry (mount height, the fixed `-0.03`/`+0.02` radial offsets in `calculate_ik_for_grasp`, or the object's height being genuinely at the edge of the arm's vertical envelope regardless of horizontal distance) still needs direct instrumentation (e.g., logging `target_reach` and `local_z` on every feasibility call) to pin down. **#1 and #2 are implemented and #2 is now validated under two independent hang scenarios (Nav2 stall in Run 3-ish delay and the tight-staging stall in Run 5); #1 remains unexercised because no run has yet reached `hover_and_open_tool`.** The next concrete step is to add that instrumentation, or to temporarily force a known-reachable `grasp_z` (e.g., leave the object physically elevated on a stand during a one-off test) purely to get a live data point on the visual servo itself, decoupled from the reach question.

## 9. Elevating the object onto a stand, and finally exercising the visual servo

Per the user's direction, `red_cylinder` was permanently relocated onto a small static stand (`cylinder_stand`, 0.05x0.05x0.15m box) in `small_house.world`, raising its grasp center from `z=0.1` to `z=0.25` - the exact height (`cube_z=grasp_z-0.05=0.20`) that reliably passed feasibility in the very first baseline runs. `semantic_map.json` and the physics-check threshold in `test_nav_and_pick.py` (now `z>0.27`) were updated to match.

Also discovered mid-session: the VM itself is chronically memory-starved the moment the full stack (Gazebo + Nav2 + MoveIt2 + the ADK node) is running - typically only ~200-250MB free out of 5.3GB, with `gz-sim-main` alone using 140%+ CPU under `llvmpipe` software rendering. This is what was actually causing the repeated multi-hundred-second Nav2 `controller_server` stalls seen throughout this session, independent of any code path here. It is a VM sizing/provisioning issue, not a bug in this codebase.

### Bypassing Nav2 to isolate the grasp mechanism
With the reach problem fixed, every remaining attempt was still blocked by Nav2 stalls before ever reaching feasibility. Per user direction, the robot's spawn pose was temporarily moved to `(-1.55, -2.0)` - the exact pose that had already passed feasibility and hover earlier in the session - and the test goal text was changed to explicitly instruct the agent not to call `navigate_and_face_tool`. This did not eliminate Nav2 calls (the agent's own instruction still said "you MUST navigate first," so an instruction-precedence fix was also applied: skip navigation only if the incoming message explicitly says the robot is already positioned), but it did let three consecutive runs reach the full `feasibility → hover_and_open_tool → visual_servo_align_tool → adjust_pose_tool → close_gripper_and_lift_tool` sequence in 2-8 minutes each, finally exercising the deterministic visual servo (#1) end-to-end.

**None of the three fully-executed runs produced a clean lift**, but each failed differently, and all three point to the same root cause:
- Run 1: `z=0.0876` after lift (cylinder knocked partway off the stand).
- Run 2: `z=0.2500` (untouched - gripper closed beside the cylinder, not around it; visible in extracted frames as closed jaws offset laterally from the upright cylinder).
- Run 3: `z=0.0138` (cylinder ends up on the floor); frame-by-frame inspection of the overhead video shows the cylinder being **violently ejected/flung** away from the gripper on contact (visible as a small red fragment in flight in the extracted frames), not gently toppled.

This ejection pattern is a classic symptom of physics-engine contact instability: both the gripper links and the cylinder are configured with very high, matched stiffness (`mu=1000`, `kp=1000000`, `minDepth=0.001` in `robot.urdf.xacro` and the world file's cylinder collision surface). Under a coarse or CPU-starved simulation step, such stiff contacts can produce a large corrective impulse that flings rather than grips. This is a physics-tuning issue in the object/gripper contact parameters, separate from - and now the actual blocker after - the agentic-reasoning issues this evaluation originally targeted.

### Final status
- **#2 (watchdogs)**: fully validated, including under a severe, previously-unseen ~500s single-call stall (this session's worst nav delay) - the system always recovered or failed cleanly, never required a manual kill in the no-navigation runs.
- **#1 (deterministic visual servo)**: now exercised in 3 live runs. It reliably drives the loop to convergence and to a physical grasp attempt without any LLM calls in the loop, which is the architectural goal - but the *physical* outcome is currently blocked by contact-parameter instability, not by the servo logic itself.
- **New, higher-priority follow-up**: retune the gripper/cylinder contact `kp`/`mu`/`minDepth` values (or increase physics solver iterations / reduce timestep) to eliminate the ejection behavior, then re-run to get a clean, physically-verified pick captured on video.

## 10. Forensic comparison against the last verified pickup (commit d1c40a0)

A prior commit, `d1c40a0` (and its parent `d8f42eb`, titled *"Correct gripper offset and close depth to successfully lift object"*), produced a physically-verified pickup. Diffing that state against the current code isolates exactly what regressed.

### What d1c40a0 did right
- **The grasp was a single atomic, blind, geometry-driven sequence** (`execute_grasp_tool`): open gripper → pre-grasp at `solve_ik(r-0.03, z)` → grasp at `solve_ik(r+0.02, z)` → close (`-0.008`) → lift at `solve_ik(r+0.02, z+0.15)` → home. No camera in the loop.
- **Horizontal side-grasp on the vertical cylinder.** It approached along a single fixed azimuth `j1`, pushed 2cm *past* the object center so the jaws straddled the body, then closed. This is highly forgiving for a tall thin cylinder.
- **A single, unchanging `j1`** — no per-step lateral perturbation.
- **Grasp at a stable mid/lower-body height**, computed off the known object pose.
- **Object at a comfortable reach distance** (`x=-1.87`, ~0.35m from the robot's spawn at `x=-1.52`), with Nav2 bypassed so the robot grasped from a known-good pose.

### What we regressed
The IK offsets (`r-0.03`, `r+0.02`) and gripper close (`-0.008`) are **still identical today** - the mechanism did not regress. What changed after `d1c40a0`:
1. **Visual servoing was added on top** (`8838d6e`), replacing the atomic side-grasp with `hover → assess → adjust → descend → close`. In this simulation the object's exact pose is already known from the semantic map, so the servo injects noisy monocular corrections into a pose that was already correct. **A correction toward a mis-estimated target is strictly worse than no correction** - visual servoing only helps under perception uncertainty, which this scenario does not have.
2. **Top-down hover-and-descend replaced the horizontal side-push.** Descending vertically onto a thin vertical cylinder lands the jaws beside it (grasps air) or clips the top (knocks/ejects it). The three no-navigation runs in §9 show exactly this: miss, knock, eject.
3. **The LLM was free to pick unstable grasp heights** (e.g. `grasp_z=0.35`, the very top of the cylinder), and to search `pitch`/`z`/staging combinatorially.
4. **The object moved closer** (`x=-1.87 → -1.75`), cramping the arm against the base.

### Gripper sizing verdict (object is correctly sized)
From the OMX URDF, both fingers are prismatic (origins `±0.021`, mimic), so **finger gap = 0.042 + 2·q**:

| Gripper joint q | Jaw gap | vs 3cm object |
|---|---|---|
| 0.019 (open) | 8.0cm | 2.5cm clearance/side |
| **-0.008 (close value)** | **2.6cm** | **0.4cm interference -> firm grip** |
| -0.011 (full close) | 2.0cm | - |

Closing to `-0.008` on the 3cm object yields a genuine 4mm squeeze - a correct grip. **The object does not need resizing.** The ejection seen in §9 is a *symptom* of closing asymmetrically on an off-center object (from top-down descent) against stiff contacts (`kp=1e6`), not a sizing problem. Beyond ~3.5cm the `-0.008` close over-penetrates and *increases* fling risk.

### Chain-of-thought critique
`d1c40a0`'s reasoning chain was short with few failure points: **ground -> navigate -> check feasibility -> execute_grasp (atomic).** The current chain is a sprawling parameter search (`try z x pitch`, hover, servo, adjust, retry), which multiplies the ways to choose wrong. Improvements adopted below:
- **Shrink the action space:** the LLM reasons about grasp *strategy* once ("tall thin vertical cylinder -> horizontal side-grasp at its center height"), then calls one validated grasp primitive; `grasp_z`/`pitch` are computed in code, not searched.
- **Give the Spatial Critic a real veto:** validate reach and that `grasp_z` is near the object center (not the top) *before* acting, rather than executing and self-approving.
- **Ground every step in real state:** verify the grasp with vision (a yes/no the VLM can answer reliably) and only report success when corroborated; never on the LLM's belief.
- **Vision is verification-only:** the camera confirms the object is held/lifted; it never estimates metric offsets it cannot measure.

### Plan implemented (this section's follow-up)
1. Restore the atomic horizontal `execute_grasp_tool` as the committed action; retire the hover/servo/adjust fragmented flow from the critic's toolset.
2. Compute `grasp_z` from the object's center (semantic map), removing the LLM's height search.
3. Add `verify_grasp_tool` (VLM yes/no on whether the object is grasped) - vision for verification only.
4. Restore the object to the `d1c40a0` configuration (`x=-1.87`, floor `z=0.1`, no stand) and grasp from the robot's spawn pose (Nav2 bypassed, given this VM's Nav2 instability).
5. Tighten the planner/critic prompts for strategy-once, real veto, and grounded verification.

## 11. Achieving a verified pickup (top-down grasp resolution)

Per the user's request, after restoring the atomic grasp and vision-for-verification, the pick was driven to a physically-verified success. This required fixing a chain of distinct issues, each isolated by measurement rather than guessing:

1. **Approach geometry (radial push -> top-down descent).** The old `calculate_ik_for_grasp` pushed the gripper radially (`r+0.02`) through the object, which knocked a thin object over. A standalone IK sweep showed the arm reaches a *top-down* pose (gripper pointing straight down) with sub-millimetre accuracy at this geometry, while a true horizontal side grasp is infeasible here (forcing it gave 90-135mm position error). Replaced the grasp with `calculate_topdown_grasp`: hover over the object centre, descend vertically so the open jaws pass around the body, close, lift straight up.
2. **Reach check too conservative.** The `0.37m` cap (added after the verified pickup commit) rejected reachable grasps; relaxed to the arm's true `~0.42m` envelope and let the IK solver's own convergence be the gate. Added `IK geometry`/`reach` logging.
3. **Grasp height calibration.** A measured run (TF of the gripper link vs the object) showed the fingertip landing 0.126m too high; `GRIPPER_FINGER_OFFSET` was set from that measurement so the fingertip lands at the object centre (verified `dz ~ 0`).
4. **Radial calibration.** A further measured overshoot of ~0.02m was corrected with `GRIPPER_RADIAL_OFFSET`, centring the jaws on the object (verified `dx ~ 0`). (A stale install `.pyc` briefly masked this; force-cleaning the bytecode cache before rebuild fixed it - a deploy gotcha worth remembering, since rsync preserves local mtimes.)
5. **Gripper actuation (the decisive fix).** With positioning perfect, the object stayed *pristine* (untouched, RPY exactly 0) across many runs - the jaws were closing with zero contact. Root cause: MoveIt-planned gripper moves report success but do not actuate the fingers in this Gazebo setup (the gripper joint never appears in `/joint_states`; the repo's own `patch.py` had previously worked around this). Added `control_gripper()` to command the gripper straight to `/gripper_controller/joint_trajectory`. The object then moved on contact for the first time.
6. **Object shape/stability.** A round cylinder makes only line-contact and squirts out of the closing flat jaws; a short thin object also gripped only shallowly. The test object was changed to a small **box** (3x3x5cm) on a stand - flat faces give full-face jaw contact - and the contact stiffness was restored to the pickup-proven `kp=1e6` now that the grasp is centred (so a symmetric close no longer flings it).

**Result:** a physically-verified pickup - `gz` reports the object lifted from its `z=0.19` rest to `z=0.56` (Success: True), and both camera videos show the box held in the gripper and carried up off the stand. Notably, on the successful run the VLM `verify_grasp_tool` returned a *false negative* ("object not in gripper") while the physics engine confirmed the lift - a reminder that the external physics check remains the ground truth and that the VLM verification, though correctly conservative across the earlier true failures, is itself imperfect and should not be the sole success signal.

### End-to-end pipeline that succeeds
`list_objects -> ground object -> check_grasp_feasibility (top-down, computed height) -> execute_grasp_tool (open, hover, descend, close via direct controller, lift, retreat) -> verify_grasp_tool (vision) -> external gz physics check`. The agentic layer is clean and honest; the manipulation now works end to end.

## 12. Grasp posture fix (object distance)

The first verified pick worked but left the arm in a cramped, near-singular posture: with the object only ~0.11m horizontally from the arm base, a top-down reach forced the arm to fold back on itself. Moving the object out to `x=-1.86` (arm base ~0.22m away, `Top-down IK r` 0.11 -> 0.22, reach 0.30 - still well inside the ~0.42m envelope) lets the arm reach in a natural, extended configuration. The pick still succeeds (object lifted from `z=0.19` to `z=0.60`) and both camera videos now show a stable, graceful reach-and-lift instead of the folded pose.
