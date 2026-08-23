# OpenFlight local API

OpenFlight exposes a local HTTP and Server-Sent Events API for companion apps
and other LAN clients. The backend owns the contract; clients must not depend
on Python, BLE, or browser implementation details.

The API currently has no authentication and is intended for a trusted local
network. Do not expose port 8080 directly to the public internet. In
particular, the legacy shutdown endpoint is intentionally not promoted into
the general versioned API.

## Version 1 resources

| Method and path | Description |
| --- | --- |
| `GET /api/v1/capabilities` | Discover the supported API and event schema versions and currently available operations. |
| `GET /api/v1/state` | Bootstrap authoritative state needed by a newly connected client. |
| `GET /api/v1/events` | Receive completed-shot and state-change events over SSE. |
| `GET /api/v1/club` | Read the Pi-owned active club. |
| `PUT /api/v1/club` | Idempotently set the active club for future shots. |
| `GET /api/v1/calibrations/iwr6843/orientation` | Read the current phone-assisted orientation calibration. |
| `PUT /api/v1/calibrations/iwr6843/orientation` | Validate, persist, and activate an orientation calibration. |

The first versioned endpoints deliberately reuse event schema V1. Versioning
the URL does not change bytes already used by released BLE and SSE clients.

## Compatibility routes

These routes remain available while existing browser and iOS clients migrate:

| Legacy route | Versioned replacement |
| --- | --- |
| `GET /api/shots/stream` | `GET /api/v1/events` |
| `GET /api/club` | `GET /api/v1/club` |
| `POST /api/club` | `PUT /api/v1/club` |
| `GET /api/calibration/iwr6843/orientation` | `GET /api/v1/calibrations/iwr6843/orientation` |
| `POST /api/calibration/iwr6843/orientation` | `PUT /api/v1/calibrations/iwr6843/orientation` |

`POST /api/shutdown` remains a legacy browser-control route. There is no
production simulate-shot REST endpoint. In mock mode, use the browser's
Socket.IO `simulate_shot` command so the generated shot traverses the normal
fan-out pipeline.

## Event stream behavior

Request the stream with `Accept: text/event-stream`. It sends an immediate
`: ping` comment and another every 15 seconds while idle. Event names are
currently `shot` and `club_changed`.

The stream replays only the latest completed shot when a client subscribes. It
does not retain an event log, emit SSE `id` fields, or honor `Last-Event-ID`, so
clients must de-duplicate shots by `event_id`. Club state is not replayed by the
stream; bootstrap it with `GET /api/v1/state`.

The default broker permits eight subscribers. Each subscriber has a queue of
eight events and drops its oldest pending event under backpressure. A ninth
subscriber receives HTTP 503.

## Backend package ownership

```text
src/openflight/api/
├── contracts.py       transport-neutral event schemas and encoders
├── dependencies.py    explicit application operations required by HTTP
└── routes.py           Flask routes and legacy compatibility aliases
```

`ble/protocol.py` owns UUIDs and binary fragmentation only. `shot_stream.py`
owns SSE buffering and framing only. Both consume `api/contracts.py`, ensuring
the domain contract is not owned by either transport.

The hardware-aware club and calibration operations still live in `server.py`
and are injected into the API route layer. They can move into application
services later without changing URLs or route tests.

## Contract change rules

- Additive optional JSON fields may remain in the current schema.
- Removing a required field, changing a field's meaning, or changing the event
  envelope requires a new event schema version.
- Keep legacy and replacement routes backed by the same application operation;
  do not fork their behavior.
- Update backend encoders, BLE and SSE adapters, canonical fixtures, client
  decoders, and compatibility tests together.
- The browser Socket.IO `{ shot, stats }` payload is a separate contract and is
  not changed by this API.
