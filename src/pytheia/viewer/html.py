"""Self-contained HTML viewer for pyTheia reconstructions.

The generated page inlines everything it needs, so it works offline and from a
plain file: three.js r160 (``three.module.min.js``), its ``OrbitControls`` and
the viewer script are embedded as text, and the scene data is embedded as JSON
with base64 ``float32``/``uint8`` buffers.

Vendored third-party files (see the license headers inside them):

* ``three@0.160.0`` ``build/three.module.min.js`` from
  ``https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js``
* ``three@0.160.0`` ``examples/jsm/controls/OrbitControls.js`` from
  ``https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js``

Only numpy is required. ``show`` needs IPython (imported lazily).
"""

from __future__ import annotations

import base64
import html as _html
import json
import re
import threading
import warnings
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

from pytheia.viewer.scene import recon_to_scene, scene_to_payload

THREE_VERSION = "0.160.0"
THREE_URL = "https://cdn.jsdelivr.net/npm/three@{}/build/three.module.min.js".format(THREE_VERSION)
ORBIT_URL = "https://cdn.jsdelivr.net/npm/three@{}/examples/jsm/controls/OrbitControls.js".format(
    THREE_VERSION
)

_ASSET_DIR = Path(__file__).resolve().parent / "assets"
_THEMES = ("dark", "light")
_PLACEHOLDER = re.compile(r"__PYTHEIA_[A-Z0-9_]+__")


def _read_asset(name: str) -> str:
    path = _ASSET_DIR / name
    if not path.is_file():
        raise FileNotFoundError(
            "pytheia viewer asset missing: {} (expected in {})".format(name, _ASSET_DIR)
        )
    return path.read_text(encoding="utf-8")


def _sanitize(obj):
    """Replace non-finite floats with None so the JSON is strict."""
    if isinstance(obj, float):
        return obj if obj == obj and obj not in (float("inf"), float("-inf")) else None
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    return obj


def _json_for_script(obj) -> str:
    text = json.dumps(_sanitize(obj), separators=(",", ":"), allow_nan=False)
    # JSON strings may contain these; escaping them keeps the tag from closing early.
    return text.replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")


def build_html(
    reconstruction,
    *,
    max_points=None,
    thumbnails_dir=None,
    include_view_graph=None,
    compare_with=None,
    title="pyTheia reconstruction",
    theme="dark",
) -> str:
    """Return the complete viewer page as a string.

    Args:
        reconstruction: A ``pytheia.sfm.Reconstruction``.
        max_points: Random subset of estimated points (``None`` keeps all).
        thumbnails_dir: Directory of per-view images, embedded as thumbnails.
        include_view_graph: Add covisibility edges (shared tracks) to the scene.
        compare_with: Second reconstruction, aligned onto ``reconstruction`` with a
            Sim(3) fit on shared view names and drawn in a second color. See :func:`compare`.
        title: Page title.
        theme: Initial theme, ``"dark"`` or ``"light"`` (toggle in the page).

    Returns:
        The HTML document as ``str``.
    """
    if theme not in _THEMES:
        raise ValueError("theme must be one of {}".format(_THEMES))
    scene = recon_to_scene(
        reconstruction,
        max_points=max_points,
        thumbnails_dir=thumbnails_dir,
        include_view_graph=include_view_graph,
        compare_with=compare_with,
    )
    payload_json = _json_for_script(scene_to_payload(scene))

    template = _read_asset("viewer.html")
    viewer_js = _read_asset("viewer.js")
    if "</script" in viewer_js.lower():
        raise RuntimeError("viewer.js must not contain a literal closing script tag")
    three_src = _read_asset("three.module.min.js")
    orbit_src = _read_asset("OrbitControls.js")

    values = {
        "__PYTHEIA_TITLE__": _html.escape(str(title), quote=True),
        "__PYTHEIA_THEME__": theme,
        "__PYTHEIA_THREE_B64__": base64.b64encode(three_src.encode("utf-8")).decode("ascii"),
        "__PYTHEIA_ORBIT_B64__": base64.b64encode(orbit_src.encode("utf-8")).decode("ascii"),
        "__PYTHEIA_SCENE_JSON__": payload_json,
        "__PYTHEIA_VIEWER_JS__": viewer_js,
    }

    def _sub(match):
        key = match.group(0)
        if key not in values:
            raise KeyError("unknown template placeholder {}".format(key))
        return values[key]

    # Single pass so that substituted content is never re-scanned.
    return _PLACEHOLDER.sub(_sub, template)


def export_html(
    reconstruction,
    path,
    *,
    max_points=None,
    thumbnails_dir=None,
    include_view_graph=None,
    compare_with=None,
    title="pyTheia reconstruction",
    theme="dark",
) -> str:
    """Write a self-contained viewer HTML file and return its absolute path.

    The file needs no network access. Open it in any modern browser.

    Args:
        reconstruction: A ``pytheia.sfm.Reconstruction``.
        path: Output file path (``.html``). Parent directories must exist.
        max_points: Random subset of estimated points (``None`` keeps all).
        thumbnails_dir: Directory of per-view images, embedded as thumbnails.
        include_view_graph: Add covisibility edges (shared tracks) to the scene.
        compare_with: Second reconstruction, aligned onto ``reconstruction`` with a
            Sim(3) fit on shared view names and drawn in a second color. See :func:`compare`.
        title: Page title.
        theme: Initial theme, ``"dark"`` or ``"light"``.
    """
    doc = build_html(
        reconstruction,
        max_points=max_points,
        thumbnails_dir=thumbnails_dir,
        include_view_graph=include_view_graph,
        compare_with=compare_with,
        title=title,
        theme=theme,
    )
    out = Path(path).expanduser().resolve()
    out.write_text(doc, encoding="utf-8")
    return str(out)


def compare(reconstruction, other, path, **kwargs) -> str:
    """Write a comparison page with two reconstructions and return its absolute path.

    ``other`` is aligned onto ``reconstruction`` with a Sim(3) (similarity) fit on
    the camera centers of the views that are registered in both under the same
    name. The two sets are drawn in different colors; each set can be toggled, and
    the view table shows the per-camera center and rotation error after alignment.
    With fewer than 3 shared views no alignment is possible: a warning is issued
    and only ``reconstruction`` is shown.

    Args:
        reconstruction: The reference ``pytheia.sfm.Reconstruction`` (frame kept).
        other: The reconstruction to align onto ``reconstruction``.
        path: Output file path (``.html``).
        **kwargs: Keyword arguments of :func:`export_html` (``max_points``,
            ``thumbnails_dir``, ``include_view_graph``, ``title``, ``theme``).
    """
    return export_html(reconstruction, path, compare_with=other, **kwargs)


def show(
    reconstruction,
    height=600,
    *,
    max_points=None,
    thumbnails_dir=None,
    include_view_graph=None,
    compare_with=None,
    title="pyTheia reconstruction",
    theme="dark",
):
    """Return an ``IPython.display.HTML`` object that embeds the viewer in a notebook.

    The page is passed as ``srcdoc`` of an iframe, so nothing is written to disk.
    Requires IPython (imported lazily).

    Args:
        reconstruction: A ``pytheia.sfm.Reconstruction``.
        height: Iframe height in pixels.
        max_points, thumbnails_dir, include_view_graph, compare_with, title, theme:
            As in :func:`export_html`.
    """
    try:
        from IPython.display import HTML
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError(
            "pytheia.viewer.show() needs IPython; install it or use export_html()"
        ) from exc

    doc = build_html(
        reconstruction,
        max_points=max_points,
        thumbnails_dir=thumbnails_dir,
        include_view_graph=include_view_graph,
        compare_with=compare_with,
        title=title,
        theme=theme,
    )
    srcdoc = _html.escape(doc, quote=True)
    frame = (
        '<iframe srcdoc="{doc}" width="100%" height="{h}" style="border:0" '
        'sandbox="allow-scripts allow-same-origin allow-downloads"></iframe>'
    ).format(doc=srcdoc, h=int(height))
    with warnings.catch_warnings():
        # IPython warns that IFrame is preferred; this iframe carries inline srcdoc, which IFrame cannot do.
        warnings.filterwarnings("ignore", message="Consider using IPython.display.IFrame")
        return HTML(frame)


def serve(
    reconstruction,
    port=8000,
    *,
    host="127.0.0.1",
    block=True,
    max_points=None,
    thumbnails_dir=None,
    include_view_graph=None,
    compare_with=None,
    title="pyTheia reconstruction",
    theme="dark",
):
    """Serve the viewer on ``http://host:port/`` with the stdlib HTTP server.

    The scene is built once, when this function is called. Use ``port=0`` to
    pick a free port; read it from ``server.server_address[1]``.

    Args:
        reconstruction: A ``pytheia.sfm.Reconstruction``.
        port: TCP port (``0`` for a free port).
        host: Interface to bind. Defaults to localhost only.
        block: If True, serve in the foreground until Ctrl-C. If False, serve
            from a daemon thread and return immediately; call
            ``server.shutdown()`` to stop.
        max_points, thumbnails_dir, include_view_graph, compare_with, title, theme:
            As in :func:`export_html`.

    Returns:
        The ``ThreadingHTTPServer`` instance.
    """
    body = build_html(
        reconstruction,
        max_points=max_points,
        thumbnails_dir=thumbnails_dir,
        include_view_graph=include_view_graph,
        compare_with=compare_with,
        title=title,
        theme=theme,
    ).encode("utf-8")

    class _Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802 - http.server naming
            if self.path.split("?", 1)[0] in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        def log_message(self, format, *args):  # keep the console quiet
            pass

    server = ThreadingHTTPServer((host, int(port)), _Handler)
    url = "http://{}:{}/".format(host, server.server_address[1])
    if block:
        print("pytheia viewer at {} (Ctrl-C to stop)".format(url))
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
    else:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
    return server
