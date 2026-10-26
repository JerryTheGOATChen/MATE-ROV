# Developing action_bringup

## Launch structure

- `action.launch.yaml` is the module's entry point, run by docker compose. It includes `haunted_house.launch.yaml` and `ghost_brain.launch.yaml`.
- `ghost_brain` runs under its own `wato_lifecycle_manager`, started by its launch file, so it can be relaunched alone during development.

## Testing

The package has no tests of its own. See [`haunted_house`](../haunted_house) and [`ghost_brain`](../ghost_brain).
