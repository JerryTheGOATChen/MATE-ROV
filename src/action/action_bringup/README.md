# action_bringup

Launch and configuration for the action module (planning and control).

On the `hackathon/halloween` branch, the module runs the [Haunted House Hackathon](../../../HACKATHON.md): `action.launch.yaml` starts the [`haunted_house`](../haunted_house) simulator and the [`ghost_brain`](../ghost_brain) starter together. The ROV's Nav2 stack has been removed from this branch, so that the ghost's planning and control are left for the hackathon. It's still on `main`.

## Usage

With `action` in `ACTIVE_MODULES`, `watod up` starts `action.launch.yaml`.

| Launch argument | Default | Description |
|-----------------|---------|-------------|
| `house_file` | `haunted_house/config/haunted_house.yaml` | The house to play |

For development, run `haunted_house.launch.yaml` and `ghost_brain.launch.yaml` in separate shells instead, so the brain can be restarted on its own. [HACKATHON.md](../../../HACKATHON.md) has the workflow.
