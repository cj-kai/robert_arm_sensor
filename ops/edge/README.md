# Edge Deployment Notes (Phase 1)

This folder is a placeholder for edge-PC deployment assets for the phase-1 stack:

- `gateway/` Node.js API + WS gateway (frontend-facing)
- `backend/` Python execution service (FSM + adapters)
- ROS 2 / FANUC driver running on the host machine

## Recommended runtime split

1. Run ROS 2 + FANUC driver on host (not containerized initially).
2. Run Python execution service on host or container.
3. Run Node gateway on host or container.
4. Open only the gateway port to clients.

## Systemd unit placeholders to add later

- `crx-gateway.service`
- `crx-exec.service`
- `crx-ros-bridge.service` (optional wrapper if needed)

