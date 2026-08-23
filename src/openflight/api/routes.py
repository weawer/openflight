"""Flask routes for the versioned API and compatibility aliases."""

from __future__ import annotations

import logging

from flask import Blueprint, Response, request

from ..shot_stream import SSE_MIMETYPE, ShotStreamFull
from .dependencies import ApiDependencies

logger = logging.getLogger(__name__)


def create_api_blueprint(dependencies: ApiDependencies) -> Blueprint:
    """Create API routes bound to explicit application operations."""
    blueprint = Blueprint("openflight_api", __name__)

    @blueprint.get("/api/v1/capabilities")
    def capabilities():
        return dependencies.capabilities()

    @blueprint.get("/api/v1/state")
    def state():
        return dependencies.state()

    @blueprint.route("/api/club", methods=["GET", "POST"])
    def legacy_club():
        if request.method == "GET":
            return dependencies.read_club()
        return dependencies.write_club(request.get_json(silent=True))

    @blueprint.route("/api/v1/club", methods=["GET", "PUT"])
    def club():
        if request.method == "GET":
            return dependencies.read_club()
        return dependencies.write_club(request.get_json(silent=True))

    calibration_path = "/api/calibration/iwr6843/orientation"

    @blueprint.route(calibration_path, methods=["GET", "POST"])
    def legacy_orientation_calibration():
        if request.method == "GET":
            return dependencies.read_orientation_calibration()
        return dependencies.write_orientation_calibration(request.get_json(silent=True))

    versioned_calibration_path = "/api/v1/calibrations/iwr6843/orientation"

    @blueprint.route(versioned_calibration_path, methods=["GET", "PUT"])
    def orientation_calibration():
        if request.method == "GET":
            return dependencies.read_orientation_calibration()
        return dependencies.write_orientation_calibration(request.get_json(silent=True))

    @blueprint.get("/api/shots/stream")
    def legacy_events():
        return _event_stream_response(dependencies)

    @blueprint.get("/api/v1/events")
    def events():
        return _event_stream_response(dependencies)

    @blueprint.post("/api/shutdown")
    def legacy_shutdown():
        return dependencies.shutdown()

    return blueprint


def _event_stream_response(dependencies: ApiDependencies):
    stream = dependencies.event_stream()
    try:
        subscriber = stream.subscribe()
    except ShotStreamFull as error:
        logger.warning("[SERVER] Refused event stream client: %s", error)
        return str(error), 503

    response = Response(stream.frames(subscriber), mimetype=SSE_MIMETYPE)
    response.headers["Cache-Control"] = "no-cache"
    response.headers["X-Accel-Buffering"] = "no"
    response.call_on_close(lambda: stream.unsubscribe(subscriber))
    return response
