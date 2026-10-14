# Developing rov_mission_runner

## Structure

- `mission.py` loads and validates mission files. Unknown keys are errors, so a typo in a criterion cannot silently disable it.
- `runner.py` sends goals through Nav2's `nav2_simple_commander` and records telemetry while they run.
- `scoring.py` turns telemetry into pass or fail checks. It is pure Python, so every criterion is unit tested.
- `main.py` is the command line entry point.

## Adding a criterion

1. Add the threshold to `Criteria` and its key to the file format in `mission.py`.
2. Record what it needs in `Telemetry`, from a callback in `runner.py`.
3. Add a check in `scoring.score()` and a failing case in `test/test_scoring.py`.
4. Document it in the README.

## Testing

```bash
colcon test --packages-select rov_mission_runner
colcon test-result --verbose
```

The unit tests cover mission files and scoring. The runner itself is exercised end to end by the acceptance test in `rov_kinematic_sim`, and by the pool missions test in `rov_gazebo_sim`.
