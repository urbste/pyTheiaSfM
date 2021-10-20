.. Theia documentation master file, created by
   sphinx-quickstart on Wed Mar  6 17:39:11 2013.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

=============
pyTheia
=============

Theia is a computer vision library initially developed by `Chris Sweeney <http://homes.cs.washington.edu/~csweeney>`_ aimed at providing efficient and reliable
algorithms for Structure from Motion (SfM). 

pyTheia provides extensive Python wrappers for Theia and actually extends the original library in several parts:

* Removed some libraries: OpenImageIO, RapidJSON, RocksDB
* Image processing and feature detection is supposed to be done in Python
* GPL SuiteSparse: Optional for ceres, however all GPL related code was removed from src/math/matrix/sparse_cholesky_llt.cc (cholmod -> Eigen::SimplicialLDLT). This will probably be slower on large problems and potentially numerically a bit more unstable.
* Added GlobalSfM methods
* Added some pose algorithms
* Added camera models (DoubleSphere, EUCM)
* Added some features to BA (e.g. pose or depth priors, covariance for points and cameras, homogeneous tangent)
* Added covariance estimation to BA

The goal of this library is still the same: provide researchers and engineers with an out of the box tool for multi-view reconstruction that can be easily extended. 
Adding the Python wrappers just makes prototyping new applications that require geometric vision or SfM capabilities quicker.

.. _section-Documentation:

=============
Documentation
=============

After building and installing the pyTheia wheel, simply import it in your Python script:

``import pytheia as pt``

You can find more examples on how to use the library in: ...

Citation
========

If you use Theia for an academic publication, please cite this
manual. e.g., ::

  @misc{theia-manual,
    author = {Chris Sweeney},
    title = {Theia Multiview Geometry Library: Tutorial \& Reference},
    howpublished = "\url{http://theia-sfm.org}",
  }

When using specific algorithms that are implemented within Theia, we ask that
you please cite the original sources. More information on which files use which
references in the literature can be found in the header files of the relevant
functions.

================
Acknowledgements
================

Theia was originally developed to provide a centralized code base to the `Four
Eyes Lab <http://ilab.cs.ucsb.edu>`_ at UC Santa Barbara, but has since been
expanded to an open-source project for the vision community.

The core of the original library is written by `Chris Sweeney
<http://homes.cs.washington.edu/~csweeney>`_. Funding for Theia was provided by his
advisors `Tobias Hollerer <http://cs.ucsb.edu/~holl>`_ and `Matthew Turk
<http://cs.ucsb.edu/~mturk>`_ in part by NSF Grant IIS-1219261, ONR Grant
N00014-14-1-0133, and NSF Graduate Research Fellowship Grant DGE-1144085.

.. toctree::
   :maxdepth: 3
   :hidden:

   building
   api
   applications
   faq
   performance
   contributions
   releases
   bibliography
   license
