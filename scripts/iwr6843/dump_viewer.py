#!/usr/bin/env python3
"""Browse IWR6843 ``.l3dump`` captures and replay the firmware against them.

    uv run python scripts/iwr6843/dump_viewer.py [--dir iwr-test-sessions] [dump.l3dump]

Serves a local page with a range x time x power surface, the range-time map
with the trigger's watch window and the tracks, trigger timelines against
their thresholds, state lanes and a per-frame inspector. Every run replays
the compiled firmware modules (``firmware_replay``) and the host ball-leave
detector (``self_trigger``) with the options set on the page; the session
log that saved a capture, when one is next to it, fills those options in.
"""

from __future__ import annotations

import argparse
import json
import threading
import webbrowser
from pathlib import Path

from flask import Flask, jsonify, request, send_file

from openflight.iwr6843.dump_viewer import ViewerOptions, analyze_dump, session_context

PAGE = Path(__file__).with_name("dump_viewer.html")
REPO = Path(__file__).resolve().parents[2]
MAX_UPLOAD_BYTES = 64 * 1024 * 1024


def create_app(root: Path) -> Flask:
    """The viewer over every ``.l3dump`` below ``root``."""
    root = root.resolve()
    app = Flask(__name__)
    app.config["MAX_CONTENT_LENGTH"] = MAX_UPLOAD_BYTES

    def resolve(relative: str) -> Path:
        path = (root / relative).resolve()
        if root not in path.parents or path.suffix != ".l3dump" or not path.is_file():
            raise ValueError(f"not a capture under {root}: {relative}")
        return path

    @app.errorhandler(ValueError)
    def bad_request(exc):
        return jsonify(error=str(exc)), 400

    @app.get("/")
    def page():
        return send_file(PAGE)

    @app.get("/api/files")
    def files():
        dumps = sorted(root.rglob("*.l3dump"), key=lambda p: p.name)
        return jsonify(
            root=str(root),
            files=[
                {"path": p.relative_to(root).as_posix(), "name": p.name, "bytes": p.stat().st_size}
                for p in dumps
            ],
        )

    @app.get("/api/context")
    def context():
        return jsonify(session_context(resolve(request.args.get("path", ""))))

    @app.post("/api/analyze")
    def analyze():
        if request.files.get("file") is not None:
            upload = request.files["file"]
            raw, name, ctx = upload.read(), upload.filename or "upload.l3dump", None
            options = json.loads(request.form.get("options") or "{}")
        else:
            body = request.get_json(force=True, silent=False) or {}
            path = resolve(str(body.get("path", "")))
            raw, name, ctx = path.read_bytes(), path.name, session_context(path)
            options = body.get("options") or {}
        result = analyze_dump(raw, ViewerOptions.from_mapping(options))
        result["name"] = name
        result["context"] = ctx
        return app.response_class(json.dumps(result, allow_nan=False), mimetype="application/json")

    return app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("dump", nargs="?", type=Path, help="Open this capture first")
    parser.add_argument(
        "--dir",
        type=Path,
        default=REPO / "iwr-test-sessions",
        help="Folder searched (recursively) for .l3dump files",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5057)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    root = args.dir
    selected = ""
    if args.dump is not None:
        dump = args.dump.resolve()
        if root.resolve() not in dump.parents:
            root = dump.parent
        selected = dump.relative_to(root.resolve()).as_posix()
    if not root.is_dir():
        raise SystemExit(f"no such folder: {root}")
    url = f"http://{args.host}:{args.port}/" + (f"#{selected}" if selected else "")
    print(f"dump viewer on {url} (captures under {root.resolve()})")
    if not args.no_browser:
        threading.Timer(1.0, webbrowser.open, args=(url,)).start()
    create_app(root).run(host=args.host, port=args.port, debug=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
