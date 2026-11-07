"""Browser viewer for pyTheia reconstructions (pure Python, numpy only).

Quick use::

    import pytheia as pt
    from pytheia import viewer

    rec = pt.sfm.Reconstruction()  # ... fill it with views, tracks and estimated poses
    viewer.export_html(rec, "scene.html")   # one self-contained file, works offline
    viewer.compare(rec, other, "diff.html") # two reconstructions, other aligned onto rec
    viewer.show(rec, height=600)            # notebook (IPython required)
    viewer.serve(rec, port=8000)            # local http server, Ctrl-C to stop

Functions:

* :func:`recon_to_scene` - numpy arrays for points, observations, cameras, edges.
* :func:`scene_to_payload` - the same data as JSON with base64 buffers.
* :func:`build_html` - the HTML document as a string.
* :func:`export_html` - write the HTML file.
* :func:`compare` - write a comparison page (Sim(3) alignment on shared views).
* :func:`show` - ``IPython.display.HTML`` iframe for notebooks.
* :func:`serve` - serve the page with ``http.server``.

The viewer embeds three.js r160 and its OrbitControls (MIT license, see the
headers of the files in ``pytheia/viewer/assets``).
"""

from pytheia.viewer.compare import align_scene, umeyama_sim3
from pytheia.viewer.html import build_html, compare, export_html, serve, show
from pytheia.viewer.scene import decode_array, recon_to_scene, scene_to_payload

__all__ = [
    "align_scene",
    "build_html",
    "compare",
    "decode_array",
    "export_html",
    "recon_to_scene",
    "scene_to_payload",
    "serve",
    "show",
    "umeyama_sim3",
]
