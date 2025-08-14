.. _chapter-io:

=============
I/O Utilities
=============

Python helpers for reading/writing reconstructions and dataset formats.

Reconstruction I/O
==================

- ``pt.io.ReadReconstruction(path)`` / ``pt.io.WriteReconstruction(recon, path)``
- ``pt.io.WriteReconstructionJson(recon, json_path)``
- ``pt.io.WritePlyFile(ply_path, recon, color=(r,g,b), point_size)``
- ``pt.io.WriteColmapFiles(recon, out_dir)``
- ``pt.io.WriteBundlerFiles(recon, lists_file, bundle_file)``
- ``pt.io.WriteNVMFile(nvm_path, recon)``

Export to Nerfstudio
====================

Writes a ``transforms.json`` compatible with Nerfstudio.

.. code-block:: python

   import pytheia as pt
   ok = pt.io.WriteNerfStudio("/path/to/images", recon, 16, "/path/to/out/transforms.json")
   # aabb_scale is typically 2, 4, 8, or 16

Notes:
- Supported camera models are mapped internally (PINHOLE and FISHEYE). Images are referenced as ``path_to_images/view_image_name``.

Export to SDFStudio
===================

Writes a JSON compatible with SDFStudio. Images must be undistorted and PINHOLE.

.. code-block:: python

   import pytheia as pt
   ok = pt.io.WriteSdfStudio("/path/to/images", recon, (2.0, 6.0), 1.0)
   # nearfar is (near, far); radius controls scene radius

Other formats
=============

- Bundler/NVM: see ``pt.io.ReadBundlerFiles``, ``pt.io.ImportNVMFile``, ``pt.io.WriteBundlerFiles``, ``pt.io.WriteNVMFile``
- COLMAP text: ``pt.io.WriteColmapFiles``