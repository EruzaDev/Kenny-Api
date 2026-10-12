# Kenny standalone API

Upload this folder as its own project. It accepts scanned ArUco IDs, resolves
their surveyed destinations, persists calls in SQLite, and lets the Raspberry
Pi claim one trip at a time. The Pi connects outward to the online API, so it
does not need an inbound public port.

```text
Android / browser scanner → HTTPS API → persistent queue ← Pi polls
                                                         ↓
                                                 model or Nav2 controller
```

The server needs Python 3.10+ and the two packages in `requirements.txt`.
It has no dependency on Kenny's editor, simulator, Torch, OpenCV, or ROS.
The model and robot controller run on the Pi, rather than on the web server.

## Start locally

```sh
cd kenny-api
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
# The included examples/environment.json is the deployed editor export.
# Load the same export on the robot and in the editor.
.venv/bin/python -m app
```

Open `http://localhost:8000/docs` for interactive endpoint documentation.
No tokens or login setup are required. The default map is
`examples/environment.json`; the queue is stored in `data/dispatch.sqlite3`.
To use another map, set `KENNY_DISPATCH_ENVIRONMENT` to its path.
The deployed export was copied from `/home/siege/environment.json` and contains
30 markers on a 240 × 160 grid using `DICT_4X4_50`. The original test map lives
in `tests/fixtures/environment.json`.

## Push to Git and deploy

Replace `examples/environment.json` with your actual surveyed map export, then
push this folder to your own Git repository. Keep that map in the repository
so it is included in the Docker image. `.env`, runtime databases, and local
Python environments are ignored by Git.

Repository: [EruzaDev/Kenny-Api](https://github.com/EruzaDev/Kenny-Api).
For a new, empty repository, run these commands from this folder:

```sh
git init
git branch -M main
git remote add origin https://github.com/EruzaDev/Kenny-Api.git
git add .
git commit -m "Prepare Kenny API for Docker deployment"
git push -u origin main
```

If the remote already contains commits, clone it first and copy this project's
files into the clone before committing and pushing.

On **Render**, create a **Blueprint**, select your repository, and use the
included `render.yaml`. It builds the Docker image, checks `/health`, and
attaches a 1 GB persistent disk for SQLite. This uses a paid service because
[Render persistent disks require a paid service](https://render.com/docs/disks).
After deployment, copy the service's HTTPS URL into the mobile app's API
settings and leave the token blank. This project stays in Python; Supabase's
[Edge Functions use TypeScript/Deno](https://supabase.com/docs/guides/functions/quickstart).

For another Python/Docker host:

- Build/install command: `pip install -r requirements.txt`
- Start command: `python -m app`, or build the included `Dockerfile`
- Health check: `/health`
- `PORT`: supplied by the host, or `8000`
- Persistent disk: set `KENNY_DISPATCH_DB` to a file on that disk
- Map: defaults to `examples/environment.json`, included in the image

Use **one service instance with a persistent disk** so restarts preserve the
queue. Multiple replicas with separate SQLite files would split calls.
Replace the map after existing trips are resolved and the Pi's navigation
map has been updated to match.

For local Docker:

```sh
cp .env.example .env
docker compose up -d --build
```

Open `http://localhost:8000/docs`. Compose exposes port 8000 and stores queue
data in a named volume. Use your hosting platform's HTTPS URL when online.

## Connect your scanner

Set the API base URL, for example `https://kenny.example.com`. Leave the token
blank. The mobile request is just a JSON POST:

```sh
curl https://kenny.example.com/api/calls \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"scan-unique-uuid","marker_id":12,"dictionary":"DICT_4X4_50","caller":"android-scanner"}'
```

Use a new `request_id` for each deliberate call and reuse it for network
retries. Only marker IDs and call metadata are sent, not camera images.
The scanner must allow an empty token; its source is outside this project.

For a browser scanner, set `KENNY_CORS_ORIGINS` to its exact origin, for
example `https://editor.example.com`. Separate multiple origins with commas.
Native Android and Pi clients do not need CORS configuration.

## Connect your Raspberry Pi

Copy `pi/receiver.py` to the Pi. Its HTTP client uses only Python's standard
library. Start it using the deployed base URL:

```sh
python3 pi/receiver.py --api https://kenny.example.com --mode model \
  --command /path/to/model-venv/bin/python /path/to/physical_controller.py
```

`--command` must come last. The receiver sends the JSON job on the controller's
stdin. The controller must accept ROS map-frame x/y in meters and yaw in
radians, navigate using its real sensor/model/motor loop, then return a single
JSON result on stdout after stopping:

```json
{"status": "succeeded", "detail": "Destination reached"}
```

Use `failed` for a stopped, unsuccessful trip. Logs go to stderr. Merely
accepting a destination or predicting an action is not success. Kenny's
existing RL checkpoint runner is a simulator; the physical sensor/motor model
controller must be supplied separately.

If Nav2 already provides physical navigation on the Pi:

```sh
source /opt/ros/jazzy/setup.bash
python3 pi/receiver.py --api https://kenny.example.com --mode nav2
```

That mode requires ROS 2 Jazzy, `nav2_simple_commander`, an active Nav2 stack,
working localization/TF, and the same surveyed map. It uses Nav2 instead of the
RL model. New calls cannot interrupt the current trip. Default trip timeout is
600 seconds; configure with `--timeout`.

## API endpoints

No Authorization header is required by default. If you want a simple token
later, set `KENNY_PI_TOKEN` to protect claim/result endpoints while leaving
mobile submission public. Set the same variable on the Pi. An optional
`KENNY_SCANNER_TOKEN` protects submit/read endpoints. A server-side
`KENNY_DISPATCH_TOKEN` sets a shared default for both; explicit role tokens
(including blank values) override it. Configured tokens use
`Authorization: Bearer TOKEN`.

| Method | Endpoint | Body / response |
|---|---|---|
| POST | `/api/calls` | `{request_id, marker_id, dictionary, caller?}` → job |
| GET | `/api/calls` | Latest 200 jobs |
| GET | `/api/calls/{id}` | Current job status |
| POST | `/api/pi/claim` | `{worker: "kenny-pi"}` → `{job: job-or-null}` |
| POST | `/api/calls/{id}/result` | `{worker, status, detail?}` → job |
| GET | `/health` | Storage health |
| GET | `/docs` | Interactive API documentation |

Example submission from a scanner:

```json
{
  "request_id": "a-new-uuid-for-this-call",
  "marker_id": 12,
  "dictionary": "DICT_4X4_50",
  "caller": "android-scanner"
}
```

The returned job includes `id`, `status`, and:

```json
{
  "destination": {"frame_id": "map", "x": 2.0, "y": 2.0, "yaw": 1.5707963267948966},
  "environment_sha256": "sha256-of-configured-map"
}
```

Calls use saved map coordinates, not coordinates supplied by the phone.
Wall markers use an arrival point 0.5 m in front of their facing direction;
floor markers use their centers. Every valid marker from the scanner's
`DICT_4X4_50`, `DICT_4X4_1000`, and `ARUCO` dictionaries is accepted and saved.
Calls without a usable mapped destination return `status: "awaiting_destination"`
and `destination: null`. This includes unknown IDs, dictionaries different from
the loaded map, and blocked or out-of-map arrival points. These calls remain
visible through the API but are not claimed by the robot. After adding a usable
destination to the map and restarting the API, submit a new scan/request ID to
queue a trip; existing waiting calls are not automatically promoted.

Retries reuse the original `request_id`; they return the original job instead
of adding another. Reusing an ID for another call returns 409. Claiming is
atomic, oldest-first, and globally limited to one running trip. The configured
service controls one Kenny robot. Request bodies are limited to 16 KB.

## Interrupted trips

The Pi journals jobs in `data/pi-receiver.json`. If a result upload fails,
it retries the upload without repeating navigation. If the Pi/controller
crashes during navigation, the API keeps that trip running and the receiver
refuses to replay it automatically after restart.

Confirm the robot has stopped, cancel any remaining controller goal, then
report the running job as `failed` using its recorded `worker` name (and token
if configured). Remove the Pi journal and restart the receiver. The queue will then
continue. A claim whose HTTP response was lost is also visible through
`GET /api/calls` and must be resolved this way. The API handles destinations;
the motor controller remains responsible for stopping the robot.

## Test

```sh
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m unittest discover -s tests -v
```

Tests exercise queue persistence, simultaneous claims, optional tokens,
public HTTP endpoints, CORS, destination validation, retries, and receiver
restart behavior without depending on the editor or simulator.
